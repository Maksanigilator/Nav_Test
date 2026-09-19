#!/usr/bin/env python3
"""Процедурные текстуры для сцены: пол, стены, потолок.

    python3 src/maze_nav/scripts/gen_textures.py -o src/maze_nav/meshes

Текстуры тут не для красоты. Камера наклонена на 30° вниз, поэтому низ
кадра всегда занимает пол — и если пол однотонный, визуальной одометрии
не за что держаться. Ровно это мы и видели на стенде: глубина на фанере
дырявая, признаков нет.

Поэтому важно КАКАЯ текстура. Шахматка или любой правильный узор хуже
однотонного: повторяющийся рисунок даёт ложные соответствия, и одометрия
не теряется, а уезжает — что гораздо коварнее. Нужен шум без периода:
крапчатый бетон, ковролин, штукатурка.

Шум складывается из нескольких масштабов (фрактальный): крупные пятна
дают признаки, которые видно издалека, мелкое зерно — те, что работают
вблизи. Одного масштаба мало: мелкий шум с двух метров сливается в серое,
крупный вблизи не даёт ни одной точки на кадр.
"""
import argparse
import pathlib

import numpy as np
from PIL import Image


def fractal(size, beta=1.9, seed=0):
    """Фрактальный шум, БЕСШОВНЫЙ по построению. Значения 0..1.

    Считаем в частотной области: случайные фазы, амплитуда падает как
    1/k**beta, обратное преобразование Фурье. Результат периодичен сам по
    себе, поэтому соседние плитки пола стыкуются без швов.

    Шов тут не косметика. Плитки на полу дают регулярную сетку, а для
    визуальной одометрии периодический рисунок ХУЖЕ однотонного: она не
    теряется, а находит ложные соответствия и уезжает. Это гораздо
    коварнее честной потери трекинга, потому что выглядит как работа.

    beta задаёт баланс масштабов: больше — крупнее пятна, меньше — мельче
    зерно. Около 1.9 получается похоже на бетон.
    """
    rng = np.random.default_rng(seed)
    k = np.fft.fftfreq(size)
    kx, ky = np.meshgrid(k, k, indexing='ij')
    r = np.hypot(kx, ky)
    r[0, 0] = 1.0                       # постоянную составляющую не трогаем
    amp = r ** (-beta / 2.0)
    amp[0, 0] = 0.0
    phase = rng.uniform(0, 2 * np.pi, (size, size))
    spec = amp * np.exp(1j * phase)
    img = np.real(np.fft.ifft2(spec))
    img -= img.min()
    return img / img.max()


def tint(noise, lo, hi):
    """Красим шум в диапазон цветов lo..hi."""
    lo, hi = np.array(lo, float), np.array(hi, float)
    return (lo + (hi - lo) * noise[..., None]).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-o', '--out', required=True, help='каталог для .png')
    ap.add_argument('--size', type=int, default=1024)
    a = ap.parse_args()
    out = pathlib.Path(a.out)

    # Пол: серый бетон с крапом. Контраст умеренный — пересвеченный пол
    # в симуляторе даёт неестественно жирные признаки, и одометрия будет
    # выглядеть лучше, чем окажется на железе.
    f = fractal(a.size, beta=1.9, seed=1)
    f = np.clip((f - f.mean()) * 2.2 + 0.5, 0, 1)
    Image.fromarray(tint(f, (86, 84, 82), (168, 166, 162))).save(out / 'floor.png')

    # Стены: светлая штукатурка, зерно мельче и контраст ниже.
    w = fractal(a.size, beta=2.3, seed=2)
    w = np.clip((w - w.mean()) * 2.6 + 0.5, 0, 1)
    Image.fromarray(tint(w, (128, 126, 120), (214, 211, 204))).save(out / 'wall.png')

    # Фанера: тёплая, с волокном. Волокно делаем анизотропным — растягиваем
    # шум вдоль одной оси. Ровный бежевый лист, каким он был раньше, давал
    # в ближней зоне кадра площадь вообще без признаков, а именно она
    # занимает низ кадра при наклоне камеры на 30°.
    g = fractal(a.size, beta=2.4, seed=3)
    g = np.array(Image.fromarray((g * 255).astype(np.uint8))
                 .resize((a.size, a.size // 8), Image.BILINEAR)
                 .resize((a.size, a.size), Image.BILINEAR), float) / 255.0
    spec = fractal(a.size, beta=1.2, seed=4)          # пятна и потёртости
    g = np.clip((g - g.mean()) * 1.8 + (spec - spec.mean()) * 0.7 + 0.5, 0, 1)
    Image.fromarray(tint(g, (150, 116, 70), (214, 180, 132))).save(out / 'plywood.png')

    for n in ('floor.png', 'wall.png', 'plywood.png'):
        p = out / n
        print(f'  {p} — {p.stat().st_size / 1024:.0f} КБ')


if __name__ == '__main__':
    main()
