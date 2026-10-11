#!/bin/bash
# run_preview_v4l2.sh - capture 10 frames and save them
python3 ~/dev/smart_chess_board/code/live_camera_preview.py \
    --device 0 \
    --interval 1.0 \
    --width 1280 \
    --height 720 &
PREVIEW_PID=$!
sleep 15
kill $PREVIEW_PID 2>/dev/null
echo ""
echo "=== Preview files saved: ==="
ls -lh /tmp/chess_preview/ 2>/dev/null || echo "No files"
echo "PREVIEW_DONE"
