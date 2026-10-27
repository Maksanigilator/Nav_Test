"""Run the unchanged step-1 detector on all saved snapshots; retain failures."""
import json
from pathlib import Path
from datetime import datetime, timedelta

import cv2
import numpy as np
import rail_hough as hough

ROOT = Path('/datasets/rail_snapshots')
COLORS = {'left': (255,180,0), 'right': (255,0,255), 'crossbar': (0,0,255)}


def partial_selection(lines, width):
    groups = {'left': [], 'right': [], 'crossbar': []}
    for line in lines:
        dx, dy = np.subtract(*line['endpoints'])
        if abs(dx) > abs(dy):
            groups['crossbar'].append(line)
        else:
            x = (line['rho']-100*np.sin(line['theta']))/np.cos(line['theta'])
            groups['left' if x < width/2 else 'right'].append(line)
    return [dict(role=role, **options[0]) for role, options in groups.items() if options]


def collage(items, path, title):
    cols, tw, th, header = 3, 636, 360, 46
    rows = (len(items)+cols-1)//cols
    canvas = np.full((64+rows*(th+header+12), cols*(tw+12)+12, 3), 24, np.uint8)
    cv2.putText(canvas, title, (18,35), cv2.FONT_HERSHEY_SIMPLEX, .8, (240,240,240), 2, cv2.LINE_AA)
    for i,(image,label,complete) in enumerate(items):
        x, y = 12+(i%cols)*(tw+12), 64+(i//cols)*(th+header+12)
        color = (120,220,140) if complete else (100,180,255)
        cv2.putText(canvas, label, (x+8,y+29), cv2.FONT_HERSHEY_SIMPLEX,.56,color,1,cv2.LINE_AA)
        canvas[y+header:y+header+th,x:x+tw] = cv2.resize(image,(tw,th),interpolation=cv2.INTER_AREA)
    if not cv2.imwrite(str(path),canvas):
        raise RuntimeError('Failed to save collage')


def main():
    output = ROOT/'hough_step1_all'
    output.mkdir(exist_ok=True)
    reports, selected_tiles, raw_tiles, skeleton_tiles = [],[],[],[]
    for i,root in enumerate(sorted(p for p in ROOT.iterdir() if (p/'mask.png').is_file()),1):
        mask = cv2.imread(str(root/'mask.png'),0)
        rgb = cv2.imread(str(root/'rgb.png'))
        sk, iterations = hough.thin(mask)
        lines = hough.candidates(sk,60)
        raw = hough.candidates(cv2.Canny(mask,50,150),60)
        error = None
        try:
            chosen = hough.select_three(lines,mask.shape[1])
        except RuntimeError as exc:
            error = str(exc)
            chosen = partial_selection(lines,mask.shape[1])
        annotated = hough.draw_lines(rgb,chosen,[COLORS[c['role']] for c in chosen],label=True)
        rawview = hough.draw_lines(rgb,raw[:40],[(0,220,255)])
        skview = hough.draw_lines(cv2.cvtColor(sk,cv2.COLOR_GRAY2BGR),chosen,[COLORS[c['role']] for c in chosen])
        dst = root/'hough_step1_batch'
        dst.mkdir(exist_ok=True)
        for name,img in [('selected.png',annotated),('raw_hough.png',rawview),('skeleton.png',sk),('selected_on_skeleton.png',skview)]:
            cv2.imwrite(str(dst/name),img)
        report = dict(sample=i,snapshot=root.name,selected=chosen,strict_three_line_success=error is None,
                      strict_error=error,missing_roles=sorted(set(COLORS)-{c['role'] for c in chosen}),
                      skeleton_candidates=len(lines),contour_candidates=len(raw),thinning_iterations=iterations,
                      parameters=dict(rho_px=1,theta_step_deg=.25,vote_threshold=60,support_band_px=3),
                      note='Selection uses the original orientation/position heuristic; 3 selected lines is not an accuracy verdict.')
        (dst/'result.json').write_text(json.dumps(report,indent=2))
        reports.append(report)
        clock = (datetime.strptime(root.name,'%Y%m%dT%H%M%SZ')+timedelta(hours=3)).strftime('%H:%M:%S')
        label=f'{i:02d} | {clock} MSK | {len(chosen)}/3 selected'
        for tiles,img in [(selected_tiles,annotated),(raw_tiles,rawview),(skeleton_tiles,skview)]:
            tiles.append((img,label,error is None))
        print(json.dumps(dict(sample=i,snapshot=root.name,selected=len(chosen),missing=report['missing_roles'])),flush=True)
    collage(selected_tiles,output/'collage_selected.png','Step 1: skeleton + Hough | cyan: left / magenta: right / red: crossbar')
    collage(raw_tiles,output/'collage_contours.png','Step 1 comparison: contour + Hough (up to 40 candidates)')
    collage(skeleton_tiles,output/'collage_skeletons.png','Step 1: skeleton and selected lines | incomplete selections retained')
    (output/'results.json').write_text(json.dumps(reports,indent=2))
    print(json.dumps(dict(total=len(reports),three_selected=sum(r['strict_three_line_success'] for r in reports),output=str(output))),flush=True)


if __name__ == '__main__':
    main()
