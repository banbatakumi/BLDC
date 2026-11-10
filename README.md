# BLDC

自律走行ミニカー用の BLDC モータドライバ (MD)。**現行のファームは [ProgramV4](ProgramV4/)** (STM32F303K8、FOC、基板 CurcuitV4)。
ProgramV1〜V3 は旧版。

ProgramV4 の文書 (パラメータの意味と根拠の正は `ProgramV4/src/app/config.h`):

| 文書 | 内容 |
|---|---|
| [docs/FOC.md](ProgramV4/docs/FOC.md) | 制御の構成・エンコーダの扱い・スイッチ校正・ゲイン・保護 |
| [docs/SERIAL_PROTOCOL.md](ProgramV4/docs/SERIAL_PROTOCOL.md) | 上位 (STM32F446RE) との UART 通信仕様 |
| [docs/HARDWARE_MAP.md](ProgramV4/docs/HARDWARE_MAP.md) | 基板のピン・相・電流センスの対応 |
| [docs/OPTIMIZATION.md](ProgramV4/docs/OPTIMIZATION.md) | 性能の設計理由・実行時間・残課題 |
