#!/usr/bin/env bash
# Собирает arm64-образы для робота и раскладывает РЯДОМ С СОБОЙ всё, что
# ему нужно: образы отдельными файлами, копию исходников проекта и
# скрипты запуска. После сборки папку docker/robot можно целиком
# скопировать на Raspberry — ни репозитория, ни интернета там не надо.
#
#   ./docker/robot/build.sh                     собрать оба образа
#   ./docker/robot/build.sh --only nav2         только навигация
#   ./docker/robot/build.sh --only yolo         только детектор знаков
#   ./docker/robot/build.sh --sync              только обновить копию src/
#   ./docker/robot/build.sh --send pi@10.0.0.5  разложить и отправить папку
#
# Образа два, а не один: torch с ultralytics весят больше, чем вся
# навигация, и их установка под эмуляцией — самое хрупкое место сборки.
# Раздельно они не мешают друг другу, и правка конфига Nav2 не тянет за
# собой пересборку детектора.
#
# Копия src/ — сознательное дублирование. Альтернатива (вшить код в
# образ) заставляла бы пересобирать arm64 под эмуляцией из-за одной
# строки в YAML, а так достаточно перегнать файл.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../.." && pwd)"

NAV2_IMAGE="nav2:jazzy-arm64"
NAV2_TAR="${HERE}/nav2-arm64.tar.gz"
YOLO_IMAGE="yolo:jazzy-arm64"
YOLO_TAR="${HERE}/yolo-arm64.tar.gz"

# Ниже есть rm -rf по вычисленному пути. Проверяем, что вычислили именно
# свою папку, а не корень проекта: опечатка тут стоила бы репозитория.
case "$HERE" in
    */docker/robot) ;;
    *) echo "build.sh должен лежать в docker/robot, а лежит в $HERE" >&2; exit 1 ;;
esac

BUILD_NAV2=1
BUILD_YOLO=1
SEND=""
while [ $# -gt 0 ]; do
    case "$1" in
        --sync) BUILD_NAV2=0; BUILD_YOLO=0; shift ;;
        --only)
            case "${2:?после --only нужно nav2 или yolo}" in
                nav2) BUILD_YOLO=0 ;;
                yolo) BUILD_NAV2=0 ;;
                *) echo "--only принимает nav2 или yolo" >&2; exit 1 ;;
            esac
            shift 2 ;;
        --send) SEND="${2:?после --send нужен адрес вида pi@10.0.0.5}"; shift 2 ;;
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "неизвестный аргумент: $1" >&2; exit 1 ;;
    esac
done

# ── Копия исходников ──────────────────────────────────────────────────
# Копируем src/ целиком, не выбирая нужные подпапки: весь каталог весит
# около 7 МБ, и любой список файлов рано или поздно отстанет от проекта.
# Зато структура на роботе ровно та же, что на ноутбуке, и одни и те же
# команды работают в обоих местах.
echo "==> копирую исходники проекта в docker/robot/src"
if command -v rsync >/dev/null 2>&1; then
    mkdir -p "${HERE}/src"
    rsync -a --delete --exclude '__pycache__/' --exclude '*.pyc' \
        "${ROOT}/src/" "${HERE}/src/"
else
    rm -rf "${HERE}/src"
    mkdir -p "${HERE}/src"
    cp -r "${ROOT}/src/." "${HERE}/src/"
    find "${HERE}/src" -name __pycache__ -type d -prune -exec rm -rf {} +
fi
du -sh "${HERE}/src"

# ── Сборка ────────────────────────────────────────────────────────────
ensure_binfmt() {
    # Без регистрации binfmt buildx умеет только amd64 и падает на первой
    # же команде внутри arm64-образа ("exec format error").
    if ! ls /proc/sys/fs/binfmt_misc/ 2>/dev/null | grep -qiE 'aarch64|arm64'; then
        echo "==> регистрирую эмуляцию arm64"
        docker run --privileged --rm tonistiigi/binfmt --install arm64
    fi
}

build_image() {
    local dockerfile="$1" image="$2" tar="$3"
    echo "==> собираю ${image} под linux/arm64 (долго: всё идёт через эмуляцию)"
    # Контекст — корень проекта: Dockerfile копирует docker/robot/entrypoint.sh.
    # --load кладёт образ ещё и в локальное хранилище, чтобы можно было
    # заглянуть внутрь через docker run --platform linux/arm64.
    docker buildx build \
        --platform linux/arm64 \
        -f "${HERE}/${dockerfile}" \
        -t "${image}" \
        --load \
        "${ROOT}"

    echo "==> выгружаю ${image} в $(basename "${tar}")"
    # gzip -1, а не по умолчанию: разница в размере проценты, а по времени
    # на пару гигабайт — минуты. docker load распознаёт gzip сам.
    docker save "${image}" | gzip -1 > "${tar}"
    ls -lh "${tar}"
}

if [ "$BUILD_NAV2" = 1 ] || [ "$BUILD_YOLO" = 1 ]; then
    ensure_binfmt
fi
[ "$BUILD_NAV2" = 1 ] && build_image Dockerfile.arm      "$NAV2_IMAGE" "$NAV2_TAR"
[ "$BUILD_YOLO" = 1 ] && build_image Dockerfile.yolo.arm "$YOLO_IMAGE" "$YOLO_TAR"

# ── Отметка о сборке ──────────────────────────────────────────────────
# Чтобы на роботе можно было ответить на вопрос «а что здесь лежит».
{
    echo "собрано:   $(date '+%Y-%m-%d %H:%M:%S')"
    echo "коммит:    $(git -C "${ROOT}" describe --always --dirty 2>/dev/null || echo '—')"
    [ -f "$NAV2_TAR" ] && echo "навигация: ${NAV2_IMAGE}, $(du -h "$NAV2_TAR" | cut -f1)"
    [ -f "$YOLO_TAR" ] && echo "детектор:  ${YOLO_IMAGE}, $(du -h "$YOLO_TAR" | cut -f1)"
} > "${HERE}/BUILD_INFO"
cat "${HERE}/BUILD_INFO"

# ── Отправка ──────────────────────────────────────────────────────────
if [ -n "$SEND" ]; then
    echo "==> отправляю папку на ${SEND}:~/nav2_robot/"
    if command -v rsync >/dev/null 2>&1; then
        # -z бесполезен для tar.gz, но исходники жмёт хорошо; --partial
        # позволяет продолжить, если Wi-Fi оборвётся на середине образа.
        rsync -a --partial --progress "${HERE}/" "${SEND}:nav2_robot/"
    else
        ssh "${SEND}" 'mkdir -p ~/nav2_robot'
        scp -r "${HERE}/." "${SEND}:nav2_robot/"
    fi
    echo "==> готово. На роботе:"
    echo "    cd ~/nav2_robot && ./run.sh          # навигация и интерфейс"
    echo "    cd ~/nav2_robot && ./run_yolo.sh     # детектор знаков"
fi
