# Self-hosted CI (Umbreon_roborace)

Активный workflow: [`.github/workflows/ci.yml`](../.github/workflows/ci.yml) — все job’ы на **`runs-on: [self-hosted, linux, embedded]`**. После успешного **firmware-build** артефакт **`firmware-pico2`** (каталог **`build/`**: `.uf2`, `.elf` и др.) доступен на вкладке **Actions** → выбранный run → **Artifacts**.

Резервная копия варианта с **GitHub-hosted** (`ubuntu-latest`): [`docs/ci-backup/ci.cloud.yml`](ci-backup/ci.cloud.yml).

---

## Обзор

Используйте тот же self-hosted runner, что и для **umbreon_zephyr** / **umbreon_esp_web**, или зарегистрируйте **отдельный** runner только для этого репозитория (см. [§ Новый runner](#инструкция-новый-runner-под-umbreon_roborace)).

Чтобы **один** физический агент обслуживал несколько репозиториев, удобен **organization-level** runner.

---

## Инструкция: новый runner под Umbreon_roborace

Ниже — **чистая** машина (VM или LXC с Ubuntu 24.04), только пользователь `runner` и GitHub Actions для репозитория **Umbreon_roborace**. Если runner уже стоит для других проектов, перейдите к [§ Регистрация на этом репо](#2-регистрация-runnerа-на-github).

### 0. Что нужно workflow

| Требование | Зачем |
|------------|--------|
| **Labels:** `self-hosted`, `linux`, `embedded` | Совпадают с `runs-on` в `ci.yml` (дополнительный `x64` не мешает). |
| **Docker** | Job **ros2-build** (Buildx + образ ROS2). |
| **`sudo` без пароля** для `runner` *или* `apt` от root | PR **style-review**: `apt install cppcheck`; job **python-import**: `apt install python3-tk` (для `matplotlib`/Tk в `plots.py`). |
| Сеть | `curl`, `pip`, Arduino CLI, кэш Docker. |

В **LXC** на Proxmox включите **`nesting=1`**, установите Docker **внутри** контейнера (см. [umbreon_zephyr/docs/self-hosted-ci.md](../../umbreon_zephyr/docs/self-hosted-ci.md), часть 1).

Если при **`docker run`** ошибка **`docker-default`** / **`apparmor_parser: Access denied`**:

1. На **хосте Proxmox** в **`/etc/pve/lxc/<VMID>.conf`**: **`lxc.apparmor.profile: unconfined`** (и **`pct` restart**). Может появиться предупреждение про **`features:nesting`** — обычно его можно игнорировать.

2. Если ошибка **остаётся**, в **гостевом** CT чаще всего помогает **убрать AppArmor** (Docker перестаёт грузить профили для контейнеров): `apt remove --purge apparmor apparmor-utils` (при необходимости после `systemctl disable --now apparmor`), затем **`systemctl restart docker`**.

3. Если **`dockerd` не стартует** после drop-in с **`--apparmor-profile=unconfined`** — удалите файл **`/etc/systemd/system/docker.service.d/apparmor.conf`**, при необходимости **`ExecStart=`** (пустая строка) только перед **одной** полной строкой **`ExecStart=`**; причина падения — **`journalctl -xeu docker.service`**. У части сборок аргумент **`unconfined`** для **dockerd** не подходит — тогда лучше п. 2, а не подбор флагов.

### 1. Система и пользователь `runner` (от root)

```bash
apt update && apt upgrade -y
apt install -y git curl ca-certificates sudo

useradd -m -s /bin/bash runner
usermod -aG dialout,plugdev runner
```

### 1.1 Docker

Пакеты **`docker-buildx-plugin`** и **`docker-compose-plugin`** в стандартном `apt` Ubuntu/Debian **часто отсутствуют** — они в репозитории **Docker Inc**. Варианты:

**Вариант A — скрипт Docker (удобнее всего: Engine + Buildx + Compose v2):**

```bash
curl -fsSL https://get.docker.com | sh
usermod -aG docker runner
systemctl enable --now docker
```

**Вариант B — только из репозитория дистрибутива (без плагинов из Docker):**

```bash
apt install -y docker.io
usermod -aG docker runner
systemctl enable --now docker
```

Проверьте, что есть **`docker compose`** (с пробелом): `docker compose version`. Если команды нет, используйте вариант A или [установите Compose v2 вручную](https://docs.docker.com/compose/install/linux/).

Проверка сессии пользователя `runner` после `docker` (нужен повторный вход):

```bash
su - runner
docker run --rm hello-world
```

### 1.2 Sudo для `cppcheck` (PR job `style-review`)

Минимально — правило для `apt`:

```bash
echo 'runner ALL=(ALL) NOPASSWD: /usr/bin/apt-get' > /etc/sudoers.d/runner-apt
chmod 440 /etc/sudoers.d/runner-apt
```

Или шире `NOPASSWD: ALL` для отладки (не рекомендуется для публичных сетей).

### 1.3 Arduino CLI (быстрее job **firmware-build**)

Если **`arduino-cli`** уже есть в **`PATH`** у пользователя **`runner`**, в workflow **не** выполняется скачивание установщика на каждый запуск.

Один раз (от root, в **`/usr/local/bin`**). Скрипт **не** принимает `-b` как в `getopt`: первый аргумент — это **тег версии** (например `1.2.3`); каталог задаётся переменной **`BINDIR`**.

```bash
sudo mkdir -p /usr/local/bin
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sudo env BINDIR=/usr/local/bin sh
```

Проверка: **`/usr/local/bin/arduino-cli version`**. У **root** в интерактивной сессии иногда нет **`/usr/local/bin`** в **`PATH`** — это нормально; для CI важнее пользователь **`runner`**: `su - runner` и затем **`command -v arduino-cli && arduino-cli version`** (на типичном Ubuntu у **`runner`** путь уже в **`PATH`**).

Кэш ядра и библиотек в **`~/.arduino15`** обрабатывается шагом **Cache** в GitHub Actions — см. [§ 1.4](#14-arduino-rp2040-и-библиотеки-на-диске-раннера-опционально).

### 1.4 Arduino rp2040 и библиотеки на диске раннера (опционально)

Один раз под пользователем **`runner`** (те же команды, что в job **firmware-build** в `ci.yml`):

```bash
su - runner
arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls \
  https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040
arduino-cli lib install "Adafruit_VL53L0X"
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "EncButton"
```

После этого шаги **Install rp2040** / **Install Arduino libraries** в CI в основном ничего не качают (ядро и libs уже в **`~/.arduino15`**).

**Почему всё ещё тянется много мегабайт:** при **попадании** в **`actions/cache`** GitHub Actions **скачивает архив кэша** с инфраструктуры GitHub (~750 MB в вашем логе) — это отдельно от «платы уже на диске». Чтобы **не** загружать этот архив, когда **`~/.arduino15`** уже заполнен локально:

1. В каталоге агента (рядом с **`runsvc.sh`**) в файле **`.env`** строка **`SKIP_ARDUINO_CACHE=true`**, затем перезапуск сервиса раннера (`sudo systemctl restart actions.runner.*` или как у вас настроено), **или**
2. В репозитории: **Settings → Secrets and variables → Actions → Variables** — переменная **`SKIP_ARDUINO_CACHE`** со значением **`true`**.

Workflow читает оба варианта (шаг **Arduino cache policy** в `ci.yml`); тогда **Cache board package** не выполняется, сборка использует локальный **`~/.arduino15`**.

### 2. Регистрация runner’а на GitHub

1. Откройте **https://github.com/&lt;OWNER&gt;/Umbreon_roborace** → **Settings** → **Actions** → **Runners** → **New self-hosted runner**.
2. Выберите **Linux** → **x64**, скопируйте **токен** из шага configure.

### 2.1 Скачать и настроить агент (под `runner`)

```bash
su - runner
mkdir -p ~/actions-runner-umbreon && cd ~/actions-runner-umbreon

RUNNER_VERSION=2.333.0
curl -O -L "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz"
tar xzf "actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz"
rm "actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz"

./config.sh \
  --url https://github.com/<OWNER>/Umbreon_roborace \
  --token <ВСТАВИТЬ_ТОКЕН> \
  --name ci-runner-umbreon \
  --labels self-hosted,linux,x64,embedded \
  --work _work
```

- **`--name`** — уникальное имя среди ваших runner’ов.
- **`--url`** — именно репозиторий **Umbreon_roborace** (если runner только для него).

### 2.2 Systemd

Выйдите из `su` и выполните от root:

```bash
cd /home/runner/actions-runner-umbreon
./svc.sh install runner
./svc.sh start
./svc.sh status
```

Логи:

```bash
journalctl -u 'actions.runner.*' -f
```

В **Settings → Actions → Runners** у репозитория **Umbreon_roborace** runner должен быть **Idle** (зелёный).

### 3. Проверка после пуша

Сделайте коммит в ветку, где включён workflow (например `main`), откройте вкладку **Actions** — job’ы должны уходить на **self-hosted**.

Первый прогон может быть долгим: скачиваются **arduino-cli**, ядро **rp2040**, библиотеки, кэш **`~/.arduino15`**, образ **ROS2**.

---

## Локальная проверка (без Actions)

```bash
make compile      # нужен arduino-cli и FQBN из Makefile
ruff check dashboard/ simulation/
make ros2-build   # Docker
```

---

## Восстановить облачный CI

Подмените содержимое `.github/workflows/ci.yml` файлом из [`docs/ci-backup/ci.cloud.yml`](ci-backup/ci.cloud.yml) (или верните `runs-on: ubuntu-latest` для всех job’ов).
