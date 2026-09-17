#!/usr/bin/env python3
"""Скачивает окружение Isaac Sim на диск, чтобы оно не тянулось по сети каждый раз.

Зачем. По умолчанию Isaac резолвит ассеты на сервер NVIDIA и тянет их во время
загрузки сцены — файл за файлом по HTTPS. Офис это 706 МБ, из них 698 МБ
текстуры в Materials/. Первый запуск из-за этого выглядит как зависание:
между "Simulation App Startup Complete" и появлением робота проходят минуты,
и в логе идёт поток предупреждений компилятора материалов, к делу не
относящихся.

Кэш в ~/.cache/ov частично спасает, но зависит от сети при каждом холодном
старте и не переносится между машинами. Локальная копия снимает вопрос совсем.

Сцены самодостаточны: office.usd ссылается только на соседние Materials/
и Props/ относительными путями, поэтому зеркало папки целиком работает
как есть.

    ./isaac/fetch_env.py office
    ./isaac/fetch_env.py --list

Скачанное ложится в isaac/assets/environments/<имя>/, и tb3_sim.py берёт
локальную копию автоматически, если она там есть.
"""
import argparse
import concurrent.futures
import pathlib
import re
import sys
import time
import urllib.parse
import urllib.request

BUCKET = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com'
ROOT = 'Assets/Isaac/5.1/Isaac/Environments/'
HERE = pathlib.Path(__file__).resolve().parent
DEST_ROOT = HERE / 'assets' / 'environments'

# Имя -> папка на сервере. Совпадает с ENVIRONMENTS в tb3_sim.py.
ENVS = {
    'office': 'Office',
    'room': 'Simple_Room',
    'warehouse': 'Simple_Warehouse',
    'shelves': 'Simple_Warehouse',
}


def get(url, timeout=60, tries=4):
    """GET с повторами: сервер ассетов периодически рвёт рукопожатие TLS."""
    for attempt in range(tries):
        try:
            return urllib.request.urlopen(url, timeout=timeout).read()
        except Exception:
            if attempt == tries - 1:
                raise
            time.sleep(2 * (attempt + 1))


def listing(prefix):
    """Все объекты под префиксом: [(ключ, размер)]."""
    out, token = [], None
    while True:
        q = {'list-type': '2', 'prefix': prefix, 'max-keys': '1000'}
        if token:
            q['continuation-token'] = token
        url = f'{BUCKET}/?{urllib.parse.urlencode(q)}'
        xml = get(url).decode('utf-8', 'ignore')
        for block in re.findall(r'<Contents>(.*?)</Contents>', xml, re.S):
            k = re.search(r'<Key>([^<]+)</Key>', block)
            s = re.search(r'<Size>(\d+)</Size>', block)
            if k and s and not k.group(1).endswith('/'):
                out.append((k.group(1), int(s.group(1))))
        m = re.search(r'<NextContinuationToken>([^<]+)</NextContinuationToken>', xml)
        if '<IsTruncated>true' in xml and m:
            token = m.group(1)
        else:
            return out


def fetch(key, size, prefix, dest):
    rel = key[len(prefix):]
    path = dest / rel
    # Уже скачанное пропускаем по совпадению размера: так повторный запуск
    # дёшев и докачивает только недостающее.
    if path.is_file() and path.stat().st_size == size:
        return 0
    path.parent.mkdir(parents=True, exist_ok=True)
    url = f'{BUCKET}/{urllib.parse.quote(key)}'
    tmp = path.with_suffix(path.suffix + '.part')
    tmp.write_bytes(get(url, timeout=180))
    tmp.rename(path)
    return size


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('env', nargs='?', choices=sorted(set(ENVS)), help='какое окружение')
    ap.add_argument('--list', action='store_true', help='показать доступные и их вес')
    ap.add_argument('--jobs', type=int, default=8, help='параллельных загрузок')
    args = ap.parse_args()

    if args.list or not args.env:
        # Папки кэшируем: warehouse и shelves лежат в одной Simple_Warehouse,
        # опрашивать её дважды незачем.
        sizes = {}
        print('окружение   папка на сервере       вес')
        for name in sorted(ENVS):
            folder = ENVS[name]
            if folder not in sizes:
                sizes[folder] = sum(s for _, s in listing(ROOT + folder + '/'))
            local = DEST_ROOT / folder
            mark = ' (есть локально)' if local.is_dir() else ''
            print(f'  {name:10s} {folder:22s} {sizes[folder]/1048576:6.0f} МБ{mark}')
        return

    folder = ENVS[args.env]
    prefix = ROOT + folder + '/'
    dest = DEST_ROOT / folder
    items = listing(prefix)
    total = sum(s for _, s in items)
    print(f'{args.env}: {len(items)} файлов, {total/1048576:.0f} МБ -> {dest}')

    done = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futs = {ex.submit(fetch, k, s, prefix, dest): s for k, s in items}
        for i, f in enumerate(concurrent.futures.as_completed(futs), 1):
            try:
                done += f.result()
            except Exception as exc:
                print(f'  ошибка: {exc}', file=sys.stderr)
            if i % 50 == 0 or i == len(items):
                print(f'  {i}/{len(items)} файлов, скачано {done/1048576:.0f} МБ',
                      flush=True)
    print(f'готово: {dest}')


if __name__ == '__main__':
    main()
