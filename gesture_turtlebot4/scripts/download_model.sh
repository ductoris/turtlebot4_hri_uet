#!/usr/bin/env bash
# Script tải mô hình MediaPipe Gesture Recognizer chính thức từ Google
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
MODEL_DIR="${PKG_DIR}/models"
MODEL_PATH="${MODEL_DIR}/gesture_recognizer.task"
MODEL_URL="https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task"

mkdir -p "$MODEL_DIR"

if [ -f "$MODEL_PATH" ]; then
    echo "[INFO] File mô hình đã tồn tại tại: $MODEL_PATH"
    echo "[INFO] Kích thước file: $(du -h "$MODEL_PATH" | cut -f1)"
else
    echo "[INFO] Đang tải mô hình gesture_recognizer.task từ Google Storage..."
    wget -c -O "$MODEL_PATH" "$MODEL_URL" || curl -L -o "$MODEL_PATH" "$MODEL_URL"
    echo "[INFO] Tải thành công vào: $MODEL_PATH"
fi

