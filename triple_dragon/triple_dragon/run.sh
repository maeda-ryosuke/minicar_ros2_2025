#!/bin/bash

# $power_mode の設定 2:パワーモード
export power_mode=2

echo "--- Jetso Power Mode & Clocks Setup ---"
sudo nvpmodel -m $power_mode

# 2回目以降は、通常、パスワード入力なしで実行される
sudo nvpmodel -q
sudo jetson_clocks

echo "--- Setup Complete ---"

python run_1219.py --model ~/notebooks/model_trt/20251219_mitaka_3stete_model_trt.pth --clsmodel ~/notebooks/model_trt/state_end_trt.pth --run --speed-mode fixed --speed-raw 110 --steering-gain 1.4
# 1210_multihead_model_trt.pth
