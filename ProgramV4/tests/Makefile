# ホストPCで走るテスト。実機に書き込まずにプロトコルと制限の論理を確認する。
#
#   make -C tests        ビルドして実行
#   make -C tests clean
#
# test_protocol : lib/crc8/crc8.h と src/app/serial_protocol.h を**そのまま**
#                 include する。ファームウェアと同じコードを検証している。
# test_limits   : bldc.c の外側ループ(位置制御)を書き写して1kHzで回す。
#                 制限とアンチワインドアップの振る舞いを見る。
#                 **bldc.c を触ったら、こちらの写しも合わせること。**
# test_calibration : src/bldc/calib_math.h を**そのまま** include し、校正の計算
#                 (補正テーブル・時定数・J/Kt・ψm) を合成データで検証する。
CC ?= cc
CFLAGS = -std=c11 -Wall -Wextra -O1 -I../lib/crc8 -I../src/app
BUILD = build

TESTS = $(BUILD)/test_protocol $(BUILD)/test_limits $(BUILD)/test_calibration

.PHONY: all run clean
all: run

run: $(TESTS)
	@for t in $(TESTS); do echo "=== $$t ==="; ./$$t || exit 1; echo; done

$(BUILD)/%: %.c | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $< -lm

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
