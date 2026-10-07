# Konami System GX — MiSTer FPGA 코어

> [!IMPORTANT]
> **취미로 만든 프로젝트입니다.**
> 개인이 순전히 취미로 만든 코어입니다. 버그 제보는 반갑게 받지만, **대응할 수도 있고 못 할 수도 있습니다.**
> 업데이트나 지원을 약속하지 않습니다.
>
> **이 코드로 이어서 개발하실 때는 꼭 출처를 남겨 주세요.**
> 이 저장소를 바탕으로 수정하거나 다른 코어를 만드실 때 출처(이 저장소 링크)를 밝혀 주시면 정말 감사하겠습니다.
> 라이선스(GPL-3.0)에 따라 원래의 저작권 표시와 이 프로젝트가 감사를 전한 분들의 출처도 함께 유지해 주세요.

1994년부터 코나미가 쓴 아케이드 기판 **System GX (Type 2)** 를 MiSTer(DE10-Nano)에서
재현하는 FPGA 코어입니다. 메인 CPU는 24 MHz **68EC020**이고, 사운드 보드에는
8 MHz **68000**, PCM 칩 **K054539** 두 개, 이펙트 DSP **TMS57002**가 있습니다.
CPU 사이에는 **K056800** 메일박스가 있습니다. 비디오는 CCU **K053252**,
타일맵 **K056832 / K054156 / K054157**(4 레이어, 5~8 bpp),
스프라이트 **K053246 / K055673**(5 bpp, 확대·축소와 그림자 포함), 우선순위 인코더
**K055555**, 최종 믹서 **K054338**로 이루어져 있습니다. 설정 저장에는 93C46
EEPROM을 씁니다. 일부 게임에 들어 있는 보호 칩 **056734 (ESC)** 는 칩이 게임
ROM 안의 자기 펌웨어를 직접 실행하는 명령어 집합 코어로 구현했습니다.
Winning Spike의 Xilinx 보호 장치도 들어 있습니다.

## 지원 게임

아래 19개 세트는 모두 2026-10-05 빌드(`gx-nineteen`)로 MiSTer 실기에서
구동을 확인했습니다. 이 배포본은 그 뒤 사운드 잡음 수정 한 가지만 더한
소스입니다(작업 내역 10-06). "실기 동작"은 실제 보드에서 부팅과 어트랙트(데모)를 거쳐
게임이 돌아간다는 뜻입니다. 같은 장면을 MAME 화면과 픽셀 단위로 비교해 남은
차이는 원인별로 분류해 두었고, 그 내용은 "알려진 제한사항"에 정리했습니다.

| MAME 세트 | 공식 명칭 | ESC | 상태 |
|---|---|---|---|
| `gokuparo` | Gokujou Parodius - Kako no Eikou o Motomete (ver JAD) | 없음 | 실기 동작 |
| `fantjour` | Fantastic Journey (ver EAA) | 없음 | 실기 동작 |
| `fantjoura` | Fantastic Journey (ver AAA) | 없음 | 실기 동작 |
| `sexyparo` | Sexy Parodius (ver JAA) | 있음 | 실기 동작 |
| `sexyparoa` | Sexy Parodius (ver AAA) | 있음 | 실기 동작 |
| `tbyahhoo` | Twin Bee Yahhoo! (ver JAA) | 있음 | 실기 동작 |
| `mtwinbee` | Magical Twin Bee (ver EAA) | 있음 | 실기 동작 |
| `daiskiss` | Daisu-Kiss (ver JAA) | 있음 | 실기 동작 |
| `salmndr2` | Salamander 2 (ver JAA) | 있음 | 실기 동작 |
| `salmndr2a` | Salamander 2 (ver AAB) | 있음 | 실기 동작 |
| `dragoonj` | Dragoon Might (ver JAA) | 있음 | 실기 동작 |
| `dragoona` | Dragoon Might (ver AAB) | 있음 | 실기 동작 |
| `crzcross` | Crazy Cross (ver EAA) | 있음 | 실기 동작 |
| `puzldama` | Taisen Puzzle-dama (ver JAA) | 있음 | 실기 동작 |
| `tokkae` | Taisen Tokkae-dama (ver JAA) | 있음 | 실기 동작 |
| `tkmmpzdm` | Tokimeki Memorial Taisen Puzzle-dama (ver JAB) | 있음 | 실기 동작 |
| `winspike` | Winning Spike (ver EAA) | Xilinx 보호 | 실기 동작 |
| `winspikea` | Winning Spike (ver AAA) | Xilinx 보호 | 실기 동작 |
| `winspikej` | Winning Spike (ver JAA) | Xilinx 보호 | 실기 동작 |

**정상 구동용 `.mra`** (MAME 세트 이름. 배포본에는 공식 명칭 파일명으로 `SD/_Arcade/_Kaze's Cores/` 에 있습니다): `gokuparo.mra`, `fantjour.mra`, `fantjoura.mra`,
`sexyparo.mra`, `sexyparoa.mra`, `tbyahhoo.mra`, `mtwinbee.mra`, `daiskiss.mra`,
`salmndr2.mra`, `salmndr2a.mra`, `dragoonj.mra`, `dragoona.mra`, `crzcross.mra`,
`puzldama.mra`, `tokkae.mra`, `tkmmpzdm.mra`, `winspike.mra`, `winspikea.mra`,
`winspikej.mra`.

개발용 `.mra` 변형(디버그 오버레이, EEPROM·ESC 끄기 등)은 이 배포본에 넣지 않았습니다.

**지원하지 않는 기판**: Type 1(`racinfrc`), Type 3(`soccerss`), Type 4(`rungun2`,
`slamdnk2`, `vsnetscr`, `rushhero`), `le2`. Type 2와 해상도·메모리 구성이 크게 달라
지금 구조로는 담을 수 없습니다.

## 설치 — ROM 만 넣으면 됩니다

이 배포본의 `SD/` 폴더는 MiSTer SD 카드의 루트(`/media/fat/`)와 같은 구조입니다.
**`SD/` 안의 내용을 SD 카드 루트에 그대로 복사**하면 됩니다.

```
SD/_Arcade/cores/KonamiGX.rbf                 코어 (빌드된 비트스트림)
SD/_Arcade/_Kaze's Cores/<게임 이름>.mra    게임 목록 (공식 명칭)
SD/games/mame/필요한_ROM.txt               넣어야 할 ROM zip 목록
```

1. `SD/` 의 내용을 SD 카드 루트에 복사합니다. 기존 파일은 덮어써도 됩니다.
2. 직접 마련한 MAME ROM 세트 zip 을 SD 카드의 `/games/mame/` 에 넣습니다.
   어떤 zip 이 필요한지는 `필요한_ROM.txt` 에 게임별로 적혀 있습니다.
   zip 은 MAME 세트 이름 그대로 두고, 압축을 풀지 않습니다.
3. MiSTer 메뉴에서 **Arcade → `_Kaze's Cores`** 로 들어가 게임을 고릅니다.

- `.rbf` 를 직접 실행하지 말고 `.mra` 로 실행하세요. ROM 로드와 DIP·OSD 기본값이
  `.mra` 에 들어 있습니다.
- 폴더 이름이 `_` 로 시작해야 MiSTer 메뉴에 보입니다. 이름을 바꾸지 마세요.

## 빌드 방법

필요한 것: **Intel Quartus Prime Lite Edition 17.0** (MiSTer 코어 표준 버전). 다른 버전에서도
합성은 될 수 있지만 검증한 버전은 17.0 입니다.

명령줄에서 빌드하려면:

```sh
cd projects/konami/konami_gx/targets/mister
quartus_sh --flow compile KonamiGX
```

결과물은 `projects/konami/konami_gx/targets/mister/output_files/KonamiGX.rbf` 입니다. Quartus GUI 로
`KonamiGX.qpf` 를 열고 Compile 을 눌러도 같습니다.

- 디렉터리 구조를 그대로 유지해야 합니다. 프로젝트 파일이 `../../../../../third_party`,
  `../../../../../platforms/mister/sys` 를 상대 경로로 찾습니다.
- `build_id.v` 는 빌드 시작 때 `platforms/mister/sys/build_id.tcl` 이 자동으로 만듭니다.
- Quartus 17.0 의 fitter 가 드물게 내부 오류로 죽으면서도 정상 종료 코드를 남기는 경우가 있습니다.
  `.rbf` 의 생성 시각과 로그 끝부분을 확인하고, 그런 경우 한 번 더 빌드하면 됩니다.

직접 빌드한 `.rbf` 는 `SD/_Arcade/cores/` 의 같은 이름 파일과 바꿔 넣으면 됩니다.

## 디렉터리 구성

원래 저장소의 상대 경로를 그대로 유지했습니다. 빌드에 실제로 쓰이는 파일만 들어 있습니다.

```
LICENSE                              GPL-3.0 전문
README.md                            이 문서
SD/                                  SD 카드 루트에 복사할 설치 파일 (RBF, MRA, ROM 목록)
projects/konami/konami_gx/
  rtl/                               기판 하드웨어 RTL (68EC020 래퍼(TG68K.C 포함), 비디오·사운드·ESC·메모리 RTL)
  integration/                       ROM 다운로드 경로 (플랫폼 중립 어댑터)
  targets/mister/                    MiSTer 최상위 (.qpf .qsf .sdc .sv files.qip, PLL)
third_party/                         외부 IP (아래 "감사의 말과 사용한 코드" 참조)
platforms/mister/sys/                MiSTer framework (Template_MiSTer)
```

소스 주석에는 개발 중에 쓴 내부 문서 번호(예: `D18`, `MEASUREMENTS 158`, `docs/...`)와
측정 기록이 그대로 남아 있습니다. 해당 개발 문서와 측정·분석 도구는 이 배포본에 포함하지
않았습니다. 주석은 설계 근거를 남기려는 것이고, 빌드에는 영향이 없습니다.

## 작업 내역

2026-09-07에 시작해 2026-10-05에 19개 세트로 마무리했습니다. 단계별로 정리하면
다음과 같습니다.

- **09-07 — 뼈대.** MAME에서 CCU 레지스터 값을 측정해 래스터(512×264, 표시
  288×224, 약 59.19 Hz)를 재현하는 K053252를 먼저 만들었습니다. 이어서
  68EC020 래퍼(TG68K.C, 68020 모드)와 주소 디코더, K056832 타일맵, 팔레트,
  K055555 비교기, K054338 믹서(jtcores `jt054338`)를 붙였습니다. 타일 페치가
  SDRAM 한 번 접근에 한 워드로는 대역폭이 모자라 버스트 읽기로 바꿨습니다.
  큰 메모리가 플립플롭으로 합성되던 문제도 블록 RAM으로 다시 써서 해결했습니다.
- **09-08 — 첫 실기 구동.** 14 MB ROM을 `.mra`로 내려받는 경로를 만들었습니다.
  이때 `.mra` 영역 사이 패딩이 빠져 그래픽이 0x120000 바이트씩 밀리던 문제와
  32비트 인터리브 순서가 뒤집혀 있던 문제를 바로잡았습니다. 93C46 EEPROM
  (jt9346)을 연결하기 전에는 게임이 EEPROM과 주고받는 신호를 MAME에서 먼저
  측정해 64×16 구성을 확인했습니다. 68EC020의 롱 접근이 두 번의 워드 접근으로
  나뉠 때 공유 읽기 포트가 첫 워드를 두 번 돌려주던 결함도 고쳤습니다.
- **09-08 ~ 09-09 — 사운드 CPU.** 메인 프로그램의 부트가 사운드 CPU의 응답을
  기다린다는 것을 확인해 K056800 메일박스와 사운드 68000(fx68k)을 넣었습니다.
  K054539 레지스터와 타이머, TMS57002 상태 비트도 함께 넣었습니다. 사운드
  CPU가 멈추던 원인은 크래시가 아니라 SDRAM 중재기에서 ROM 읽기가 굶는
  것이었습니다.
- **09-12 ~ 09-14 — 타일맵과 스프라이트.** 타일 표시 원점, 레이어별 X 오프셋,
  플립, 플레인 바이트 순서를 MAME와 맞췄습니다. K055673 스프라이트는 vblank DMA,
  zcode 정렬 스캔, 5 bpp 라인 버퍼, 그림자, 타일 단위 확대·축소로 구현했습니다.
  화면이 복잡할 때 스프라이트 줄이 늦게 그려지던 문제는 블록 RAM 스프라이트
  행 캐시와 8줄 라인 버퍼 링으로 해결했습니다.
- **09-15 — 속도와 소리.** 메인 CPU ROM 캐시, 2워드 버스트, 플레인 4 타일
  캐시를 넣어 데모 진행 속도를 MAME의 약 100 %로 맞췄습니다. K054539 PCM
  엔진과 SDRAM 샘플 페치를 붙였고, 리버브 링은 DDR3로 옮겼습니다. TMS57002는
  MAME의 명령어 정의를 따라 RTL로 옮긴 뒤 MAME 코드를 그대로 컴파일한 모델과
  샘플 단위로 대조했습니다. 실기에서 녹음한 출력이 모델과 스펙트럼·음높이·
  레벨 모두 일치했습니다. 같은 날 Gokujou Parodius를 실기에서 직접 플레이해
  확인했습니다.
- **09-16 ~ 09-22 — Fantastic Journey와 Sexy Parodius.** Fantastic Journey의
  0xDB0000 DMA 장치를 두 번째 버스 마스터로 구현했습니다. Sexy Parodius가 부팅
  중 RAM CHECK에서 멈추던 원인은 두 가지였습니다. 사운드 쪽 지연 RAM이 없었고,
  메인 CPU가 실제보다 약 20 % 빨랐습니다. 지연 RAM을 넣고 CPU 클럭을 보정해
  부팅을 통과했습니다. 스프라이트 테이블을 zcode 계수 정렬로 바꾸고 그림자
  우선순위를 PCB 영상에 맞춘 뒤, MAME 대조 16장이 모두 일치했습니다.
- **09-28 ~ 09-29 — Twin Bee Yahhoo!.** IRQ1 동기 비트, 겹치는 그림자 처리,
  캐시가 꺼진 동안 POST 지연을 재현하는 처리를 넣어 어트랙트 40/40 일치를
  얻었습니다. 코어가 돌고 있는 상태에서 리셋해도 DDR3가 깨끗하게 멈추도록
  고쳤습니다.
- **09-29 ~ 10-02 — ESC(056734) 보호 칩.** 각 칩의 부트 코드와 커널을 명령어
  수준 모델로 실행해 다섯 타이틀의 ESC 프로그램을 복호했습니다. 처음에는
  프로그램을 옮겨 적은 회로로 구동했고, 이후 칩이 게임 ROM의
  펌웨어를 직접 실행하는 명령어 집합 코어(`rtl/esc/`)로 교체했습니다. 이 코어는
  명령어 모델과 부팅부터 11,115개 RUN 프레임까지 메모리 전체가 일치합니다.
  메인 CPU는 칩이 계산하는 동안 멈추지 않고 계속 실행합니다.
- **09-30 ~ 10-01 — Salamander 2와 Dragoon Might.** 32 MB ROM 맵을 하나로
  통일했습니다. 스프라이트 ROM 형식 세 가지(GX 5 bpp, 6 bpp, 4 bpp)와 6 bpp 타일,
  IRQ3(오브젝트 DMA 종료), 레이어별 밝기 프리셋을 넣었습니다. EEPROM 시작
  비트를 실제 93C46처럼 레벨로 받도록 고쳐 Dragoon Might의 EEPROM 오류도
  없앴습니다.
- **10-02 ~ 10-03 — 정리와 확장.** 드물게 사운드 RAM CHECK가 실패하던 TMS57002
  리셋 결함을 고쳤습니다. 세트 이름으로 정하던 타일·스프라이트 X 오프셋은 게임이
  쓰는 CCU 레지스터에서 계산하도록 바꿨습니다. Crazy Cross, Taisen Puzzle-dama,
  Taisen Tokkae-dama, Tokimeki Memorial Taisen Puzzle-dama를 추가했습니다.
- **10-05 — Winning Spike, 19개 세트로 마무리.** 8 bpp 타일 행 버스트와
  Winning Spike의 Xilinx 보호 동작을 넣었고, 19개 세트 모두 실기 구동을
  확인했습니다.
- **10-06 — 사운드 잡음 수정.** 극상파로디우스와 섹시파로디우스에서 들리던
  지지직 잡음은 한 샘플만 크게 튀는 출력이었습니다(실기 녹음 87초에 약 2,000개).
  TMS57002가 계수 갱신(cload)이 시작될 때마다 진행 중이던 명령을 하나씩 버리고
  있었습니다. 버리는 조건을 MAME와 같이 프로그램 로드(pload) 시작으로만 줄여,
  튀는 샘플이 정상 수준(87초에 2~5개, 음악 자체의 급변)으로 돌아왔습니다.

- **10-06 — 게임별 분기 제거와 CCU.** 스프라이트 DMA 뱅크, 데이터 ROM 창, 3P 입력이 게임 이름으로
  켜지고 꺼지던 것을 없애고, 하드웨어 레지스터와 주소 디코드만 따르게 했습니다. 바꾸기 전에 MAME 로
  19개 세트를 모두 돌려 결과가 같다는 것을 확인했습니다. 화면 타이밍 칩(K053252)은 Furrtek 의
  실리콘 재구성과 같은 레지스터 값으로 나란히 돌려 비교했고, 수평 동기 시작이 1 dot 다른 것을
  실리콘에 맞췄습니다(픽셀은 변하지 않음). 실기 6개 세트에서 새 차이는 없었습니다.

## 감사의 말과 사용한 코드

이 코어는 **MAME 팀** 덕분에 만들 수 있었습니다. MAME의 Konami GX 드라이버와
각 칩의 디바이스 소스는 메모리 맵부터 비디오 칩 동작, 사운드 칩 테이블까지
이 이식의 동작 기준이었습니다. 오랜 세월 아케이드 하드웨어를 기록하고 보존해
온 모든 MAME 기여자께 깊이 감사드립니다.
— <https://www.mamedev.org/> · <https://github.com/mamedev/mame>

### 이 릴리스에 포함된 외부 코드

| 이름 | 용도 | 저자 | 라이선스 | 출처 (링크 + 커밋) | 사용한 파일 | 수정 여부 |
|---|---|---|---|---|---|---|
| TG68K.C | 메인 CPU 68EC020 (`CPU="11"`, 68020 모드) | Tobias Gubener (MikeJ, Till Harbaum, Rok Krajnk 외 패치, 파일 머리말 표기) | LGPL-3.0-or-later | <https://github.com/TobiFlex/TG68K.C> @ `ade33e396a1e647c2de9daf71ff9d5b3979639b2` (줄끝 정규화 후 내용 일치로 식별) | `rtl/cpu/tg68k/TG68K_Pack.vhd`, `TG68K_ALU.vhd`, `TG68KdotC_Kernel.vhd` (`TG68K.vhd`는 함께 두지만 빌드에 쓰지 않음) | 없음 |
| fx68k | 사운드 CPU 68000 | Jorge Cwik | GPL-3.0-only | <https://github.com/ijor/fx68k> @ `0602ee4627b10f301298f2673d826cdd6baa9327` | `third_party/cpu/fx68k/`: `fx68k.sv`, `fx68kAlu.sv`, `uaddrPla.sv`, `microrom.mem`, `nanorom.mem` | 있음: `fx68k.sv`에 읽기 전용 디버그 포트 `dbg_d7`를 추가함 (파일 안 `// LOCAL:` 주석) |
| jt054338 | K054338 최종 믹서 레지스터 | Jose Tejada Gomez (jotego) | GPL-3.0-or-later | <https://github.com/jotego/jtcores> @ `62cacc840340ad1fe7d6be481d0f9bbebe835e7d` (`cores/moo/hdl/jt054338.v`) | `third_party/video/jt054338/jt054338.v` | 없음. 알파 0 처리 결함은 원본을 고치지 않고 `rtl/video/gx_colmix.sv`에서 우회 |
| jt9346 | 93C46 직렬 EEPROM (64×16) | Jose Tejada Gomez (jotego) | GPL-3.0-or-later | <https://github.com/jotego/jteeprom> @ `9c68ce841f4ec560ca6f228c8af6301129fd95fa` (`hdl/jt9346.v`) | `third_party/peripheral/jt9346/jt9346.v` | 없음. 시작 비트 처리는 앞단 `rtl/gx_eestart.sv`에서 보정 |
| jt053246_scan, jtframe_draw (파생) | K053246/K055673 스프라이트 라인 스캔과 16도트 타일 그리기 | Jose Tejada Gomez (jotego) | GPL-3.0-or-later | <https://github.com/jotego/jtcores> @ `62cacc840340ad1fe7d6be481d0f9bbebe835e7d` (`cores/simson/hdl/jt053246_scan.sv`, `modules/jtframe/hdl/video/jtframe_draw.v`, `jtframe_objdraw_gate.v`의 LATCH 입력단) | `rtl/video/gx_objscan.sv`, `rtl/video/gx_objdraw.sv` | 있음 (파생 구현). 5 bpp, zcode 범위, 플립, 확대·축소 연산, 빈 슬롯 건너뛰기 등을 바꿨고 변경 목록은 각 파일 머리말에 있음 |
| MiSTer framework (`sys/`) | MiSTer 보드 인터페이스: HPS I/O, OSD, 스케일러, HDMI, 오디오 출력 | MiSTer-devel 기여자 (Sorgelig 외) | 파일별 (대부분 GPLv2+ 또는 GPLv3+, 각 파일 머리말 기준) | <https://github.com/MiSTer-devel/Template_MiSTer> @ `54ac838e019d7fa07fbb40677a104cd6620d15c3` (2026-08-17, `sys/` 내용 일치로 식별) | `platforms/mister/sys/*` | 있음: `sys.tcl`에서 `build_id.tcl` / `sys.qip` 경로를 스크립트 위치 기준으로 계산하도록 2줄 수정 (동작 변화 없음) |

**감사드립니다.**

- **Jose Tejada Gomez (jotego)** — jtcores와 jteeprom은 코나미 칩 재현의 든든한
  토대였습니다. 믹서, EEPROM, 스프라이트 스캔 코드를 가져다 썼고, 그 밖의 모듈도
  하드웨어 사실을 확인하는 데 큰 도움이 됐습니다.
- **Jorge Cwik** — 사이클 정확한 68000 코어 fx68k 덕분에 사운드 CPU를 걱정 없이
  맡길 수 있었습니다.
- **Tobias Gubener와 TG68K.C 기여자들** — 68020 명령어 집합을 온전히 갖춘 TG68K.C가
  없었다면 68EC020 메인 CPU는 훨씬 먼 길이었을 것입니다.
- **MiSTer-devel과 Sorgelig, 그리고 MiSTer 커뮤니티** — 이 코어가 올라가는 플랫폼
  전체를 만들고 지켜 주셨습니다.

### 참고했지만 코드는 가져오지 않은 자료

- **Furrtek 의 K053252 실리콘 재구성** (jtcores `cores/rungun/doc/053252.v`) — 이 코어의 CCU 와 같은
  레지스터 값으로 나란히 시뮬레이션해 타이밍을 대조하는 데만 썼습니다. 파일에 라이선스 표기가 없어
  코드는 가져오지 않았습니다. 실리콘을 분석해 공개해 주신 Furrtek 님께 감사드립니다.
- **jtcores의 다른 모듈** — `modules/jt05415x`(Furrtek의 K054156/K054157 실리콘
  재구성), `cores/rungun/hdl/jtk053252.v`, `cores/moo/hdl/jtmoo_colmix.v`,
  `jt053246.sv`, `jt053246_dma.v`, `jt053246_mmr.v`, `jtframe_obj_buffer.v`는
  하드웨어 사실을 확인하는 용도로만 읽었습니다. 코드는 복사하지 않았습니다.
- **FBNeo** — FBNeo에는 Konami GX 드라이버가 없습니다. FBNeo 코드는 이 코어에
  한 줄도 들어 있지 않습니다.
- **ESC(056734)** — 칩의 펌웨어는 사용자가 제공하는 게임 ROM에 들어 있고, 코어가
  실행 중에 그것을 읽어 실행합니다. 펌웨어 자체는 이 배포본에 없습니다. 초기에 쓰던, 복호한 프로그램을 회로로 옮겨 적은
  구현(`gx_esc.sv`, 지금 빌드에서는 쓰지 않음)은 게임 데이터를 담고 있어 배포본에서 뺐습니다. 명령어
  의미는 자체 명령어 모델로 정했고, MAME의 ESC 처리 결과와 스프라이트 데이터를
  대조해 검증했습니다.

### MAME 참고 파일

MAME 소스 기준 커밋은 `446356f29ee59b4f2dad4f93408b1aaa33fae926`입니다. RTL
주석에서 `konamigx.cpp`를 `gx.cpp`로, `konamigx_v.cpp`를 `gxv.cpp`로 줄여 쓴
곳이 있습니다.

| MAME 파일 | 라이선스 | copyright-holders | 참고한 내용 |
|---|---|---|---|
| `src/mame/konami/konamigx.cpp` | BSD-3-Clause | R. Belmont, Acho A. Tang, Phil Stroffolino, Olivier Galibert | 메인·사운드 CPU 메모리 맵, 클럭(XTAL과 분주), 인터럽트, 화면 파라미터, ROM 구성, 입출력 포트, 세트별 머신 설정, EEPROM 종류, TMS57002 상태 비트, Winning Spike 보호(`type4_prot_w`), POST 지연 처리 |
| `src/mame/konami/konamigx_v.cpp` | BSD-3-Clause | R. Belmont, Acho A. Tang, Phil Stroffolino, Olivier Galibert | 레이어 합성 순서, K055555 / K053247 색 결합, 그림자, 알파 반전, 레이어 오프셋 |
| `src/mame/konami/konamigx_m.cpp` | BSD-3-Clause | R. Belmont, Acho A. Tang, Phil Stroffolino, Olivier Galibert | Fantastic Journey의 0xDB0000 DMA 장치 |
| `src/mame/konami/k054156_k054157_k056832.cpp` | BSD-3-Clause | David Haywood | 타일맵 레지스터, VRAM 창, 스크롤, 라인 스크롤 |
| `src/mame/konami/k053246_k053247_k055673.cpp`, `.h` | BSD-3-Clause | David Haywood | 스프라이트 속성 해석, 확대·축소 연산(`zdrawgfxzoom32GP`), 그림자 |
| `src/mame/konami/k055555.cpp` | BSD-3-Clause | David Haywood | 우선순위 인코더 레지스터와 동작 |
| `src/mame/konami/k054338.cpp`, `.h` | BSD-3-Clause | David Haywood | 믹서 레지스터, 밝기와 블렌드 |
| `src/mame/konami/konami_helper.cpp` | BSD-3-Clause | David Haywood | 스프라이트 색 단위와 펜 처리 |
| `src/mame/konami/mystwarr_v.cpp` | BSD-3-Clause | R. Belmont, Phil Stroffolino, Acho A. Tang, Nicola Salmoria | 알파 반전 설정 비교 (System GX와의 차이 확인) |
| `src/devices/machine/k053252.cpp` | LGPL-2.1+ | Angelo Salese | CCU 레지스터 맵 |
| `src/devices/sound/k054539.cpp`, `.h` | BSD-3-Clause | Olivier Galibert | PCM 엔진, 볼륨·팬 테이블, 타이머, 리버브 |
| `src/devices/sound/k056800.cpp`, `.h` | BSD-3-Clause | Ville Linde | 사운드 메일박스 레지스터와 인터럽트 |
| `src/devices/cpu/tms57002/tms57002.cpp`, `.h`, `tms57kdec.cpp`, `tmsinstr.lst`, `tmsmake.py` | BSD-3-Clause | Olivier Galibert | TMS57002 명령어 정의, 디코드 순서, 호스트 포트, 실행 흐름 |
| `src/emu/diexec.cpp` | BSD-3-Clause | Aaron Giles | 리셋 중 실행 정지(suspend) 동작 |
| `src/emu/disound.h` | BSD-3-Clause | Aaron Giles, Olivier Galibert | 사운드 스트림 출력값의 정수 변환 |

`tmsinstr.lst`에는 라이선스 머리말이 없습니다. 같은 디렉터리의 `tmsmake.py`
머리말을 기준으로 표기했습니다.

MAME에서 동작을 확인했지만 실제 PCB 회로가 확인되지 않은 부분이 있습니다.
K054539 볼륨 테이블, 세트별 오프셋, Winning Spike의 Xilinx 보호 등입니다. 이런
부분은 MAME의 동작을 그대로 따르며, RTL 주석에 `EMULATION_DERIVED` 또는
`APPROXIMATION`으로 표시해 두었습니다.

## 라이선스

이 코어는 전체적으로 **GPL-3.0**으로 배포합니다. GPL-3.0-only인 fx68k가 들어
있기 때문입니다. TG68K.C(LGPL-3.0-or-later)와 jotego 모듈(GPL-3.0-or-later)도
GPL-3.0과 함께 쓸 수 있습니다. 저장소의 `LICENSE` 파일은 GPL-3.0 전문입니다.

외부 파일은 각자의 라이선스와 저작권 머리말을 그대로 유지합니다. MiSTer
framework(`sys/`)는 파일마다 머리말에 적힌 라이선스를 따릅니다.

**ROM 파일은 포함되어 있지 않습니다.** 게임 ROM은 사용자가 직접 마련해야 합니다.
`.mra`는 MAME ROM 세트(zip) 이름을 기준으로 ROM을 찾습니다.

## 알려진 제한사항

- **System GX Type 2 기판만 지원합니다.** Type 1, 3, 4와 `le2`(광선총, 화면 상하
  반전, 8 MB 타일)는 지원하지 않습니다.
- **메인 CPU가 사이클 단위로 정확하지는 않습니다.** TG68K.C는 원본 68EC020과
  사이클 타이밍이 다르고 데이터 버스도 16비트입니다. 진행 속도는 CPU 클럭
  인에이블을 보정해 MAME에 맞췄습니다. 캐시가 꺼진 동안의 POST 지연은 68020
  명령어 사이클 비율로 근사했습니다(`APPROXIMATION`).
- **사운드 CPU 작업 RAM은 PCB의 64 KB 창 중 16 KB만 구현했습니다.** 지원하는
  모든 세트에서 실제로 쓰는 범위를 측정해 정했고, 소리와 성능에는 차이가
  없습니다. K054539 리버브 링과 TMS57002 외부 메모리는 MiSTer의 DDR3에 둡니다.
- **MAME 대조에서 남은 차이가 있습니다.** 원인은 분류되어 있습니다. 대표적으로
  MAME가 게임의 VRAM 갱신 도중에 찍은 스냅샷 차이(Salamander 2의 태양), 가산
  블렌드 극성(PCB 영상과 일치하는 쪽을 택함), 진행 속도 차이로 생기는 프레임
  어긋남이 있습니다. Winning Spike는 8장 중 2장이 정확히 일치했고 나머지도 분류된
  원인에 속합니다.
- **Tokimeki Memorial Taisen Puzzle-dama의 KONAMI 로고 밝기**가 페이드 정점에서
  코어는 흰색, MAME는 회색입니다. 어느 쪽이 PCB와 같은지는 아직 모릅니다.
- **OSD "Voice boost"** 는 MAME의 Dragoon Might 칩 2 게인 보정을 옵션으로 둔
  것입니다. 기본값은 꺼짐이고 모든 세트에 적용됩니다.
- **ESC 세부 사항 일부는 추론입니다.** 부팅 순서, 인터럽트 펄스 같은 일부 동작은
  커널 코드와 참고 자료에서 추론했고 PCB에서 확인한 것은 아닙니다. Salamander 2의
  ESC는 bin 번호가 7을 넘으면 하위 3비트로 감쌉니다(`APPROXIMATION`, 실제
  플레이에서는 관측되지 않음).
- **타이밍 여유가 작습니다.** 출하 빌드의 코어 클럭 여유는 +0.062 ns입니다.
  소스를 고치거나 Quartus 시드를 바꾸면 타이밍을 놓칠 수 있습니다.
- **Analogue Pocket은 지원하지 않습니다.**
