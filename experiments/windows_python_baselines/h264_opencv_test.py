"""
Standalone test: does this machine's OpenCV support H.264 encoding,
and how fast is it? Run this before wiring anything into ROS.

No ROS dependency -- just cv2. Captures from webcam, writes N frames
to a local H.264 file via the Windows Media Foundation backend, then
times decode/read-back for comparison against your JPEG numbers.
"""
import time
import cv2

DEVICE_INDEX = 0
NUM_FRAMES = 100
OUTPUT_FILE = "h264_test.mp4"

def main():
    cap = cv2.VideoCapture(DEVICE_INDEX)
    if not cap.isOpened():
        print(f"Failed to open camera at index {DEVICE_INDEX}")
        return

    ret, frame = cap.read()
    if not ret:
        print("Failed to grab a test frame")
        return
    h, w = frame.shape[:2]
    fps = 15.0

    # Try Windows Media Foundation backend explicitly -- this uses the
    # OS-native H.264 encoder, sidestepping the missing-libx264 issue
    # in pip opencv-python's bundled ffmpeg.
    fourcc = cv2.VideoWriter_fourcc(*'H264')
    writer = cv2.VideoWriter(OUTPUT_FILE, cv2.CAP_MSMF, fourcc, fps, (w, h))

    if not writer.isOpened():
        print("MSMF + H264 failed to open. Trying 'avc1' fourcc instead...")
        fourcc = cv2.VideoWriter_fourcc(*'avc1')
        writer = cv2.VideoWriter(OUTPUT_FILE, cv2.CAP_MSMF, fourcc, fps, (w, h))

    if not writer.isOpened():
        print("H.264 encoding is NOT available on this OpenCV build/backend.")
        print("Falling back to 'mp4v' (MPEG-4 Part 2, NOT true H.264) just")
        print("so you have a working comparison point -- results won't")
        print("reflect real H.264 performance.")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(OUTPUT_FILE, fourcc, fps, (w, h))
        if not writer.isOpened():
            print("Even mp4v failed to open. Something is more broken than codecs.")
            cap.release()
            return

    print(f"VideoWriter opened successfully -> {OUTPUT_FILE}")
    print(f"Encoding {NUM_FRAMES} frames at {w}x{h}...")

    encode_times = []
    for i in range(NUM_FRAMES):
        ret, frame = cap.read()
        if not ret:
            break
        t0 = time.perf_counter()
        writer.write(frame)
        t1 = time.perf_counter()
        encode_times.append((t1 - t0) * 1000.0)

    writer.release()
    cap.release()

    if encode_times:
        avg = sum(encode_times) / len(encode_times)
        print(f"\nEncode: {len(encode_times)} frames, avg {avg:.2f} ms/frame")
        print(f"Effective encode-only FPS: {1000.0/avg:.1f}")

    # Now time decode/read-back of what we just wrote
    print(f"\nReading back {OUTPUT_FILE} to measure decode time...")
    read_cap = cv2.VideoCapture(OUTPUT_FILE)
    if not read_cap.isOpened():
        print("Could not reopen the written file for decode timing -- "
              "the file may not have been finalized correctly.")
        return

    decode_times = []
    while True:
        t0 = time.perf_counter()
        ret, frame = read_cap.read()
        t1 = time.perf_counter()
        if not ret:
            break
        decode_times.append((t1 - t0) * 1000.0)
    read_cap.release()

    if decode_times:
        avg = sum(decode_times) / len(decode_times)
        print(f"Decode: {len(decode_times)} frames, avg {avg:.2f} ms/frame")
        print(f"Effective decode-only FPS: {1000.0/avg:.1f}")

if __name__ == '__main__':
    main()
