# AReuMii-Teleop

IsaacLab 전체 설치 없이 양팔과 그리퍼의 XR 입력을 IK 관절 목표로 변환해
UDP bridge로 전송하는 독립 패키지입니다. Linux x86_64 / Python 3.12를 대상으로 합니다.

시뮬레이션, 데이터셋 도구, VLA 추론·평가는
[AReuMii-VLA](https://github.com/happpps/AReuMii-VLA)에서 관리합니다.
이 저장소는 실기 XR teleop과 UDP bridge를 관리합니다.

```text
Quest / CloudXR → XR controller → URDF IK → UDP
    → C++ bridge → Areum2_can System V shared memory → SocketCAN → motors
```

## 설치와 설정 확인

이 디렉터리가 독립 저장소의 루트입니다. IsaacLab 저장소 안에서는 먼저
`cd real_teleop`을 실행합니다. Python 패키지와 C++ bridge 모두 상위 저장소의
파일이나 `/home/ys/...` 경로를 요구하지 않습니다.

```bash
# XR 없이 패키지와 모델/설정만 확인
uv sync --frozen
uv run --no-sync areumii-teleop --print_config

# 실제 XR 실행에 필요한 패키지 설치
uv sync --frozen --extra runtime
uv run --no-sync areumii-teleop --help
```

`uv.lock`은 전체 의존성을 고정합니다. 실행 환경을 동기화할 때는 `--extra runtime`을
포함하고, 동기화 후 실행할 때는 위처럼 `--no-sync`를 사용합니다.
LeRobot 0.6.1, IsaacTeleop 1.4.98rc1, Placo 0.9.15를 기준으로 구성했습니다.
NVIDIA 인덱스는 `pyproject.toml`에 지정되어 있습니다.

CloudXR 런타임 설치와 최초 EULA 동의는 별도로 진행합니다.
기존 런타임을 외부에서 실행한다면 `LEROBOT_CLOUDXR_SKIP_AUTOLAUNCH=1`을 사용합니다.
상세 설치는 [Isaac Teleop 공식 문서](https://nvidia.github.io/IsaacTeleop/)를 확인합니다.

## 실행

호환되는 시뮬레이션 수신기를 먼저 시작한 뒤 실행합니다.

```bash
uv run --no-sync areumii-teleop --mode sim
```

기본값은 명령 송신 비활성입니다. 설정/피드백 확인 후 아래처럼 명시적으로 활성화합니다.

```bash
uv run --no-sync areumii-teleop --mode sim --enable_command
```

실기에서는 호스트에서 기존 `Areum2_can` 프로그램과 아래 C++ bridge를 실행한 뒤
`--mode real`로 실행합니다. 양쪽 squeeze는 각 팔의 clutch, trigger는 각 그리퍼를
제어합니다. 실제 명령 송신은 `--enable_command`가 있어야 활성화됩니다.

| 모드 | 명령 포트 | 피드백 포트 |
| --- | --- | --- |
| sim | 5010 | 5011 |
| real | 5005 | 5006 |

시뮬레이션 수신기 `test_real_teleop_in_sim.py`는 **IsaacLab 환경에서만** 실행합니다.
독립 패키지가 IsaacLab을 설치하거나 포함하지는 않습니다. 시뮬레이션용 로봇 asset과
task도 IsaacLab 쪽 환경에 필요합니다.

## URDF와 설정 파일

기본 모델은 `src/areumii_teleop/assets/areumii_c1_kinematics.urdf`이며 wheel에도
포함됩니다. 설치 위치와 현재 작업 디렉터리에 관계없이 자동으로 찾습니다.
루트의 같은 이름 URDF는 이전 명령을 위한 상대 심볼릭 링크입니다.

`configs/default.toml`은 패키지 기본 설정을 가리킵니다. 수정할 설정은
`configs/local.example.toml`을 복사해 작성합니다.

```bash
cp configs/local.example.toml configs/local.toml
uv run --no-sync areumii-teleop --config configs/local.toml --print_config
uv run --no-sync areumii-teleop --config configs/local.toml --mode real
```

우선순위는 **CLI > 사용자 TOML > 패키지 기본값**입니다. TOML 안의 URDF·CloudXR
경로는 TOML 파일 위치를 기준으로, CLI의 경로는 현재 디렉터리를 기준으로 해석합니다.
`--urdf /path/to/model.urdf`로 모델을 바꿀 수도 있습니다. 관절 이름·말단 프레임 이름이
모델과 일치하는지 실행 전에 검증합니다.

설정에서 주파수 [Hz], 관절·그리퍼 각도 [rad], 이동량 [m], 피드백 제한 시간 [s]을
지정합니다. 명령 송신 활성화는 TOML에 넣을 수 없고 CLI에서만 지정합니다.
현재 bridge의 그리퍼 허용 범위는 C++ 코드에 별도로 정의되어 있으므로 Python
보정값을 바꿀 때 함께 확인해야 합니다.

새 CLI는 snake_case를 사용하며, 이전 `--enable-command`, `--cmd-port` 같은
옵션도 유지합니다. 기존 `areumii_bimanual_teleop_gripper.py` 실행 경로도 유지합니다.
`areumii_real_teleop.py`와 `areumii_bimanual_teleop.py`는 이전 실행 방식으로 보존한
스크립트이며, 새 기능은 통합된 `areumii-teleop` 명령에서 사용합니다.

## C++ bridge 빌드

```bash
cmake -S can_bridge -B build/bridge -DCMAKE_BUILD_TYPE=Release
cmake --build build/bridge --parallel
# Areum2_can을 실행한 후 bridge 시작
./build/bridge/areumii_bimanual_udp_shm_bridge_gripper
```

현재 include는 폴더 내부 `can_bridge/inc/Sharemem.hpp`를 사용하므로 외부 include
절대 경로가 필요하지 않습니다. 이 헤더는 CAN 프로그램과 메모리 배치·모터 수가
일치해야 합니다. 모터 수는 16, System V key는 13563267입니다.
헤더가 다른 CAN 프로그램과는 먼저 ABI 일치를 확인해야 합니다.

Bridge는 `L`/`R` + 7개 관절 목표, `GL`/`GR` + 그리퍼 목표를 받고,
28개 또는 32개 값의 피드백을 Python에 보냅니다. 그리퍼 제어에는 32개 피드백이 필요합니다.
기존 recorder 피드백 포트 5007을 사용하는 대체 bridge도 함께 빌드됩니다.

## Docker

```bash
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm teleop
```

기본 컨테이너 명령은 `--print_config`라 장치를 시작하지 않습니다. Docker도
동일한 `uv.lock`으로 runtime extra를 설치합니다. 실제 이미지 빌드 및 XR 장치
연결은 대상 머신에서 추가 검증해야 합니다.

초기 구성에서는 CAN과 bridge는 호스트에 두고 Python teleop만 컨테이너로 옮깁니다.
Compose의 `network_mode: host`는 Linux에서 호스트 UDP bridge 접근에 사용합니다.
CloudXR 자동 실행은 꺼져 있습니다. 실제 XR 실행에는 해당 머신의 OpenXR 런타임에
맞는 환경변수, 런타임 파일/IPC 소켓, GPU/장치 접근 구성이 추가로 필요합니다.
이 compose 파일만으로 XR 연결이 완료되는 것은 아닙니다.

Bridge까지 옮길 경우 System V 공유 메모리를 위해 호스트 IPC namespace 공유 여부도
검토해야 합니다. 현재 Python 컨테이너에는 `ipc: host`, `privileged`를 요구하지 않습니다.

## 검증과 GitHub 업로드용 소스

```bash
uv run --no-sync python -m unittest discover -s tests -v
uv build --no-sources
uv run --no-sync python tools/export_source.py
```

`dist/areumii-teleop-source.tar.gz`를 풀면 독립 저장소로 올릴 수 있는 폴더가 나옵니다.
현재 IsaacLab의 git index와 remote는 변경하지 않습니다. 업로드할 저장소에는
`pyproject.toml`, `uv.lock`, 소스와 설정 예시, bridge, Docker, 라이선스를 포함합니다.
ZIP, 환경 파일, outputs, 로컬 설정, 빌드 바이너리는 배포용 소스에서 제외합니다.
기존 상위 저장소에서 이미 추적하거나 스테이징한 파일은 `.gitignore`로 자동 해제되지
않으므로 상위 저장소 전체를 그대로 올리는 것과 구분해야 합니다.

기본 테스트는 모델/설정의 이동 가능성과 UDP 프로토콜을 검증합니다. 실제 XR 세션,
CAN 연결, 모터 동작은 하드웨어 통합 검증 대상으로 남습니다.

## 라이선스

원본 저작권 헤더를 유지합니다. `LICENSE`, `NOTICE`, `licenses/Apache-2.0.txt`를
함께 배포합니다. XR helper의 기존 Apache-2.0 헤더와 외부 런타임의 라이선스도 확인합니다.
