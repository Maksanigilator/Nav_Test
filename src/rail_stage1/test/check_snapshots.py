"""Offline replay; supplied camera TF may differ from TF at capture time."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from rail_stage1.geometry import rotation, floor_plane, bird_view, thin, line_candidates, select_three


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('camera_tf', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config/rails.yaml').read_text())['rail_finder']['ros__parameters']
    tf = json.loads(args.camera_tf.read_text())
    r, origin = rotation(tf['quaternion']), np.array(tf['translation'])
    args.output.mkdir(parents=True, exist_ok=True)
    reports, tiles = [], []
    for sample in sorted(p for p in args.root.iterdir() if (p/'mask.png').exists()):
        meta = json.loads((sample/'metadata.json').read_text())
        k = np.array(meta['camera_info']['k']).reshape(3, 3)
        distortion = np.array(meta['camera_info']['d'])
        mask = cv2.imread(str(sample/'mask.png'), 0)
        depth = np.load(sample/'depth.npy').astype(float)*.001
        row = dict(snapshot=sample.name, accepted=False)
        view = np.zeros((300, 320, 3), np.uint8)
        try:
            if np.count_nonzero(mask) < cfg['min_mask_area_px']:
                raise ValueError('mask below minimum area')
            n, d = floor_plane(depth, mask, k, distortion, r, origin, cfg)
            bev, plane_origin, ex, ey = bird_view(mask, k, distortion, r, origin, n, d, cfg)
            view = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
            lines = line_candidates(thin(bev), cfg)
            row['candidates'] = len(lines)
            point, direction, selected = select_three(lines, cfg)
            row.update(accepted=True, entry_base=(plane_origin+point[0]*ex+point[1]*ey).tolist(),
                       direction_base=(direction[0]*ex+direction[1]*ey).tolist())
            def pixel(p):
                return (int(round((p[0]-cfg['bev_x_min'])/cfg['bev_resolution'])),
                        int(round((p[1]+cfg['bev_half_width'])/cfg['bev_resolution'])))
            for line, color in zip(selected, [(255, 180, 0), (255, 0, 255), (0, 0, 255)]):
                cv2.line(view, pixel(line['a']), pixel(line['b']), color, 2)
            cv2.circle(view, pixel(point), 5, (0, 255, 0), -1)
            cv2.arrowedLine(view, pixel(point-.9*direction), pixel(point), (255, 160, 0), 2)
        except (ValueError, cv2.error) as exc:
            row['reason'] = str(exc)
        cv2.imwrite(str(args.output/(sample.name+'.png')), view)
        tile = np.full((340, 420, 3), 25, np.uint8)
        cv2.putText(tile, sample.name, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (230, 230, 230), 1)
        cv2.putText(tile, '3 lines' if row['accepted'] else row['reason'][:48], (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, .40, (120, 220, 140) if row['accepted'] else (100, 160, 255), 1)
        tile[50:330, 10:410] = cv2.resize(view, (400, 280))
        tiles.append(tile)
        reports.append(row)
    canvas = np.full((50+340*((len(tiles)+2)//3), 1260, 3), 25, np.uint8)
    cv2.putText(canvas, 'Stage 1 offline replay | current camera TF, not recorded TF', (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX, .7, (230, 230, 230), 1)
    for i, tile in enumerate(tiles):
        canvas[50+(i//3)*340:50+(i//3+1)*340, (i%3)*420:(i%3+1)*420] = tile
    cv2.imwrite(str(args.output/'collage.png'), canvas)
    result = dict(note='Uses current camera TF; original snapshots did not record TF. No map pose validation.',
                  total=len(reports), accepted=sum(row['accepted'] for row in reports), samples=reports)
    (args.output/'results.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
