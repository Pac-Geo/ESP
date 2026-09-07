#!/usr/bin/env python3
# ---------------------------------
# Title: Raspberry Pi Photo Frame
# ---------------------------------
# Author: Jorge PalomecNicolas
# Date: September 6, 2026
# Versions:
#   v1.0 - Initial Raspberry Pi full-screen photo viewer
#   v2.0 - USB hard-drive storage + Google Drive Apps Script sync
# -----------------------------------------------------------
# Setup:
#   Raspberry Pi 3 Model B+
#   Raspberry Pi OS
#   HDMI display / monitor
#   USB hard drive mounted at /media/pacgeo/CUADRO
#
# Interpreter:
#   Python 3
#
# File Dependencies / Libraries:
#   pygame
#   Pillow (PIL)
#   requests
#   VLC (external video player)
#
# Required packages:
#   sudo apt install -y python3-pygame python3-pil python3-requests vlc ffmpeg
#
# Purpose:
#   Full-screen digital photo frame for Raspberry Pi 3B+.
#   Photos are stored on the USB hard drive and synchronized
#   from Google Drive through the same Apps Script bridge used
#   by the previous ESP32-S3 implementation.
#
# Inputs:
#   - Google Drive photo list/download data through Apps Script
#   - Image files on USB hard drive
#   - Keyboard: Left/Right, R=rescan, S=sync, Esc/Q=exit
#
# Outputs:
#   - Full-screen HDMI slideshow
#   - Aspect-correct foreground image
#   - Blurred same-image background
#   - Terminal sync/download/status messages
# -----------------------------------------------------------

import base64
import hashlib
import os
import re
import sys
import time
from pathlib import Path

import pygame
import requests
import subprocess
import threading
import subprocess
from PIL import Image, ImageFilter, ImageOps

USB_LABEL = 'CUADRO'
USB_MEDIA_BASE = Path('/media/pacgeo')

def resolve_usb_root() -> Path:
    '''
    Raspberry Pi OS may mount the same drive as CUADRO, CUADRO1,
    CUADRO2, etc. Find the currently accessible CUADRO* directory
    automatically instead of hard-coding one mount path.
    '''
    candidates = []

    exact = USB_MEDIA_BASE / USB_LABEL
    if exact.exists() and exact.is_dir():
        candidates.append(exact)

    try:
        for p in sorted(USB_MEDIA_BASE.glob(f'{USB_LABEL}*')):
            if p.is_dir() and p not in candidates:
                candidates.append(p)
    except OSError:
        pass

    for candidate in candidates:
        try:
            next(candidate.iterdir(), None)
            return candidate
        except OSError:
            continue

    return exact

USB_ROOT = resolve_usb_root()
PHOTO_DIR = USB_ROOT / 'PHOTOS'
VIDEO_CACHE_DIR = USB_ROOT / '.photo_frame_cache' / 'videos'

APPS_SCRIPT_URL = (
    'https://script.google.com/macros/s/'
    'AKfycbzke5eN-5VNLriVYGjG8mH4-dEWb1km7iGoxhgm262S62JwzOVulGjC9ajhb3DQ8ZdRJA/'
    'exec'
)
APPS_SCRIPT_TOKEN = 'facildeconectar'

SLIDE_SECONDS = 10
SYNC_INTERVAL_SECONDS = 15 * 60
DOWNLOAD_CHUNK_SIZE = 16 * 1024
MAX_PHOTO_SIZE = 100 * 1024 * 1024
DOWNLOAD_RETRIES = 3
HTTP_TIMEOUT_SECONDS = 20


# Background Drive sync state
sync_lock = threading.Lock()
sync_thread = None
sync_in_progress = False
sync_completed_generation = 0

IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}
VIDEO_EXTENSIONS = {'.mp4', '.m4v', '.mov', '.avi', '.mkv', '.mpeg', '.mpg', '.webm', '.mts', '.m2ts', '.3gp', '.wmv', '.mvre'}
SUPPORTED_EXTENSIONS = IMAGE_EXTENSIONS | VIDEO_EXTENSIONS
DRIVE_MIME_TYPES = {
    'image/jpeg', 'image/png',
    'video/mp4', 'video/quicktime', 'video/x-msvideo',
    'video/x-matroska', 'video/mpeg', 'video/webm'
}
session = requests.Session()


def usb_is_mounted():
    """Return True when the CUADRO path is present and readable."""
    try:
        if not USB_ROOT.exists() or not USB_ROOT.is_dir():
            return False
        next(USB_ROOT.iterdir(), None)
        return True
    except (OSError, PermissionError):
        return False


def ensure_photo_directory():
    if not usb_is_mounted():
        print(f'USB drive path is not accessible at: {USB_ROOT}')
        return False
    try:
        PHOTO_DIR.mkdir(parents=True, exist_ok=True)
        return True
    except OSError as exc:
        print(f'Could not create/access {PHOTO_DIR}: {exc}')
        return False


def sanitize_filename(name):
    cleaned = re.sub(r'[\/\\:*?"<>|\x00-\x1F]', '_', str(name)).strip()[:220]
    return cleaned or 'UNNAMED.JPG'


def find_photos():
    if not ensure_photo_directory():
        return []
    try:
        photos = [
            p for p in PHOTO_DIR.iterdir()
            if p.is_file()
            and p.suffix.lower() in SUPPORTED_EXTENSIONS
            and p.name != 'DOWNLOAD.TMP'
        ]
    except OSError as exc:
        print(f'Could not scan photo folder: {exc}')
        return []
    return sorted(photos, key=lambda p: p.name.lower())


def file_matches_size(path, expected_size):
    try:
        return path.is_file() and path.stat().st_size == expected_size
    except OSError:
        return False


def request_json(params, retries=DOWNLOAD_RETRIES):
    last_error = None
    for attempt in range(1, retries + 1):
        try:
            response = session.get(
                APPS_SCRIPT_URL,
                params=params,
                timeout=HTTP_TIMEOUT_SECONDS,
                allow_redirects=True,
            )
            response.raise_for_status()
            payload = response.json()
            if not payload.get('success', False):
                raise RuntimeError(f'Apps Script returned error: {payload}')
            return payload
        except Exception as exc:
            last_error = exc
            print(f'HTTP attempt {attempt}/{retries} failed: {exc}')
            if attempt < retries:
                time.sleep(0.5)
    raise RuntimeError(f'Apps Script request failed: {last_error}')


def fetch_drive_photo_list():
    print('\nRequesting Google Drive photo list...')
    payload = request_json({'action': 'list', 'token': APPS_SCRIPT_TOKEN})
    photos = []
    for item in payload.get('photos', []):
        try:
            photos.append({
                'id': str(item['id']),
                'name': str(item['name']),
                'mimeType': str(item['mimeType']),
                'size': int(item['size']),
            })
        except (KeyError, TypeError, ValueError):
            print(f'Skipping malformed Drive entry: {item}')
    print(f'Drive photos parsed: {len(photos)}')
    return photos


def fetch_drive_chunk(photo, offset, requested_length):
    payload = request_json({
        'action': 'chunk',
        'id': photo['id'],
        'offset': offset,
        'length': requested_length,
        'token': APPS_SCRIPT_TOKEN,
    })
    returned_offset = int(payload['offset'])
    raw_length = int(payload['length'])
    if returned_offset != offset:
        raise RuntimeError(f'Chunk offset mismatch: requested {offset}, got {returned_offset}')
    decoded = base64.b64decode(payload['data'], validate=True)
    if len(decoded) != raw_length:
        raise RuntimeError(f'Chunk length mismatch: expected {raw_length}, got {len(decoded)}')
    return decoded


def download_photo(photo):
    if photo['mimeType'] not in DRIVE_MIME_TYPES:
        return False
    size = int(photo['size'])
    if size <= 0 or size > MAX_PHOTO_SIZE:
        print(f"Skipping size outside limit: {photo['name']}")
        return False
    if not ensure_photo_directory():
        return False

    safe_name = sanitize_filename(photo['name'])
    final_path = PHOTO_DIR / safe_name
    if file_matches_size(final_path, size):
        return True

    temp_path = PHOTO_DIR / 'DOWNLOAD.TMP'
    try:
        temp_path.unlink(missing_ok=True)
    except OSError:
        pass

    print(f"\nDownloading: {photo['name']}")
    print(f'Size: {size} bytes')
    offset = 0
    last_percent = -1

    try:
        with temp_path.open('wb') as output:
            while offset < size:
                request_length = min(DOWNLOAD_CHUNK_SIZE, size - offset)
                chunk = fetch_drive_chunk(photo, offset, request_length)
                if not chunk:
                    raise RuntimeError('Received an empty chunk')
                output.write(chunk)
                offset += len(chunk)
                percent = (offset * 100) // size
                if percent != last_percent and (percent % 5 == 0 or percent == 100):
                    print(f'Download {percent}%')
                    last_percent = percent
            output.flush()
            os.fsync(output.fileno())

        if temp_path.stat().st_size != size:
            raise RuntimeError('Downloaded byte count verification failed')

        os.replace(temp_path, final_path)
        print(f'Saved: {final_path}')
        return True
    except Exception as exc:
        print(f"Download FAILED for {photo['name']}: {exc}")
        try:
            temp_path.unlink(missing_ok=True)
        except OSError:
            pass
        return False


def sync_drive_to_usb():
    print('\n========================')
    print('GOOGLE DRIVE -> USB SYNC')
    print('========================')

    if not ensure_photo_directory():
        print('Cannot sync: CUADRO USB drive is not mounted.')
        return False

    try:
        photos = fetch_drive_photo_list()
    except Exception as exc:
        print(f'Drive list request failed: {exc}')
        return False

    downloaded = skipped = failed = 0
    for photo in photos:
        size = int(photo['size'])
        if size <= 0 or size > MAX_PHOTO_SIZE or photo['mimeType'] not in DRIVE_MIME_TYPES:
            skipped += 1
            continue

        final_path = PHOTO_DIR / sanitize_filename(photo['name'])
        if file_matches_size(final_path, size):
            print(f'Already on USB: {final_path.name}')
            skipped += 1
            continue

        if download_photo(photo):
            downloaded += 1
        else:
            failed += 1

    print('\n========================')
    print('SYNC COMPLETE')
    print('========================')
    print(f'Downloaded: {downloaded}')
    print(f'Skipped:    {skipped}')
    print(f'Failed:     {failed}')
    return failed == 0


def _background_sync_worker():
    global sync_in_progress, sync_completed_generation

    try:
        sync_drive_to_usb()
    finally:
        with sync_lock:
            sync_in_progress = False
            sync_completed_generation += 1


def start_background_sync():
    """Start Drive -> USB synchronization without blocking the slideshow."""
    global sync_thread, sync_in_progress

    with sync_lock:
        if sync_in_progress:
            print("Google Drive sync is already running in the background.")
            return False

        sync_in_progress = True
        sync_thread = threading.Thread(
            target=_background_sync_worker,
            name="drive-sync",
            daemon=True,
        )
        sync_thread.start()

    print("Google Drive sync started in background.")
    return True


def pil_to_surface(image):
    image = image.convert('RGB')
    return pygame.image.fromstring(image.tobytes(), image.size, 'RGB')


def is_video(path):
    return path.suffix.lower() in VIDEO_EXTENSIONS


def _video_cache_path(path):
    stat = path.stat()
    identity = f"{path.resolve()}|{stat.st_size}|{stat.st_mtime_ns}"
    digest = hashlib.sha1(identity.encode("utf-8")).hexdigest()[:12]
    safe_stem = re.sub(r"[^A-Za-z0-9._-]+", "_", path.stem).strip("._") or "video"
    return VIDEO_CACHE_DIR / f"{safe_stem}_{digest}_pi.mp4"


def prepare_video_for_pi(path):
    """
    Create a Pi 3B+-friendly H.264/AAC playback copy while keeping the
    original video untouched.

    This version targets smooth playback first:
      - max 1280x720
      - max 30 fps
      - H.264 yuv420p
      - CRF 17 for good visual quality
      - fast-decode-friendly H.264 settings
    """
    VIDEO_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cached = _video_cache_path(path)

    if cached.exists() and cached.stat().st_size > 0:
        return cached

    temp = cached.with_suffix(".tmp.mp4")
    temp.unlink(missing_ok=True)

    print()
    print(f"Preparing smooth Pi playback copy: {path.name}")
    print("Original file is NOT modified.")
    print(f"Playback cache: {cached}")

    vf = (
        "scale=w='min(1280,iw)':h='min(720,ih)':"
        "force_original_aspect_ratio=decrease:"
        "flags=lanczos,"
        "scale=trunc(iw/2)*2:trunc(ih/2)*2,"
        "fps=30"
    )

    command = [
        "ffmpeg", "-y",
        "-hide_banner",
        "-loglevel", "warning",
        "-i", str(path),
        "-map", "0:v:0",
        "-map", "0:a:0?",
        "-vf", vf,
        "-c:v", "libx264",
        "-preset", "veryfast",
        "-tune", "fastdecode",
        "-crf", "17",
        "-pix_fmt", "yuv420p",
        "-profile:v", "high",
        "-level", "3.1",
        "-x264-params", "ref=2:bframes=2",
        "-maxrate", "6000k",
        "-bufsize", "12000k",
        "-c:a", "aac",
        "-b:a", "160k",
        "-ac", "2",
        "-movflags", "+faststart",
        str(temp),
    ]

    result = subprocess.run(command)

    if result.returncode != 0 or not temp.exists() or temp.stat().st_size == 0:
        temp.unlink(missing_ok=True)
        raise RuntimeError(
            f"ffmpeg could not create a compatible playback copy for {path.name}"
        )

    os.replace(temp, cached)
    print(f"Video conversion complete: {cached.name}")
    return cached


def get_video_duration_seconds(path):
    """Return video duration in seconds, or None if ffprobe cannot read it."""
    try:
        result = subprocess.run(
            [
                "ffprobe",
                "-v", "error",
                "-show_entries", "format=duration",
                "-of", "default=noprint_wrappers=1:nokey=1",
                str(path),
            ],
            capture_output=True,
            text=True,
            check=False,
        )

        if result.returncode != 0:
            return None

        return float(result.stdout.strip())
    except Exception:
        return None


def play_video(path):
    """
    Play with VLC while keeping the pygame slideshow full-screen underneath.

    Q is treated as a full application quit:
      1. VLC receives Q and closes the current video.
      2. We detect that the video ended early.
      3. The main slideshow exits instead of continuing.
    """
    playable = prepare_video_for_pi(path)
    duration = get_video_duration_seconds(playable)

    print(f"Playing video: {path.name}")
    print("Press Q during a video to exit the entire photo frame.")

    started = time.monotonic()

    process = subprocess.Popen(
        [
            "cvlc",
            "--fullscreen",
            "--play-and-exit",
            "--no-video-title-show",
            "--quiet",
            "--key-quit=q",
            str(playable),
        ]
    )

    try:
        return_code = process.wait()
    except KeyboardInterrupt:
        print("\nExit requested. Stopping video...")
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        raise

    elapsed = time.monotonic() - started

    print(
        f"Video finished: {path.name} "
        f"(VLC exit {return_code}, played {elapsed:.1f}s)"
    )

    if return_code != 0:
        raise RuntimeError(f"VLC failed with exit code {return_code}")

    # If VLC closed substantially before the media duration, assume Q was used.
    # Allow a small margin because startup/metadata timing is not exact.
    if duration is not None and elapsed < max(1.0, duration - 1.5):
        print("Early video exit detected -> closing entire photo frame.")
        return True

    return False


def make_frame(path, screen_w, screen_h):
    with Image.open(path) as src:
        image = ImageOps.exif_transpose(src).convert('RGB')

        background = image.copy()
        cover_scale = max(screen_w / background.width, screen_h / background.height)
        background = background.resize(
            (
                max(1, round(background.width * cover_scale)),
                max(1, round(background.height * cover_scale)),
            ),
            Image.Resampling.LANCZOS,
        )
        left = max(0, (background.width - screen_w) // 2)
        top = max(0, (background.height - screen_h) // 2)
        background = background.crop((left, top, left + screen_w, top + screen_h))
        background = background.filter(ImageFilter.GaussianBlur(radius=24))
        black = Image.new('RGB', background.size, (0, 0, 0))
        background = Image.blend(background, black, 0.18)

        foreground = image.copy()
        foreground.thumbnail((screen_w, screen_h), Image.Resampling.LANCZOS)
        x = (screen_w - foreground.width) // 2
        y = (screen_h - foreground.height) // 2
        background.paste(foreground, (x, y))

        return pil_to_surface(background)


def draw_message(screen, lines):
    screen.fill((0, 0, 0))
    font = pygame.font.Font(None, 38)
    y = 50
    for line in lines:
        surface = font.render(line, True, (255, 255, 255))
        screen.blit(surface, (50, y))
        y += 48
    pygame.display.flip()


def main():
    print('\n========================')
    print('RASPBERRY PI PHOTO FRAME')
    print('========================')
    print(f'USB drive: {USB_ROOT}')
    print(f'Photo directory: {PHOTO_DIR}')
    print('CUADRO USB drive mounted.' if usb_is_mounted() else 'WARNING: CUADRO USB drive is NOT mounted.')

    # Start sync in the background so any media already on the USB drive
    # can begin displaying immediately.
    start_background_sync()

    pygame.init()
    pygame.mouse.set_visible(False)
    info = pygame.display.Info()
    screen_w, screen_h = info.current_w, info.current_h
    print(f'Detected display: {screen_w}x{screen_h}')

    screen = pygame.display.set_mode((screen_w, screen_h), pygame.FULLSCREEN | pygame.DOUBLEBUF)
    pygame.display.set_caption('Raspberry Pi Photo Frame')

    photos = find_photos()
    seen_sync_generation = sync_completed_generation
    index = 0
    current_surface = None
    current_path = None
    last_slide_change = 0.0
    last_sync = time.monotonic()
    clock = pygame.time.Clock()
    running = True

    while running:
        now = time.monotonic()
        force_reload = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_RIGHT and photos:
                    index = (index + 1) % len(photos)
                    force_reload = True
                elif event.key == pygame.K_LEFT and photos:
                    index = (index - 1) % len(photos)
                    force_reload = True
                elif event.key == pygame.K_r:
                    print('Rescanning USB photo directory...')
                    photos = find_photos()
                    index = 0
                    current_surface = None
                    current_path = None
                    force_reload = True
                elif event.key == pygame.K_s:
                    print('Manual Google Drive sync requested.')
                    if start_background_sync():
                        last_sync = time.monotonic()

        if now - last_sync >= SYNC_INTERVAL_SECONDS:
            print('Automatic Google Drive sync requested...')
            if start_background_sync():
                last_sync = time.monotonic()

        # If a background sync finished, refresh the media list without
        # interrupting the currently displayed photo/video.
        if sync_completed_generation != seen_sync_generation:
            seen_sync_generation = sync_completed_generation
            updated_photos = find_photos()

            if updated_photos != photos:
                current_name = current_path.name if current_path is not None else None
                photos = updated_photos

                if current_name is not None:
                    for i, media_path in enumerate(photos):
                        if media_path.name == current_name:
                            index = i
                            break
                    else:
                        index = min(index, max(0, len(photos) - 1))
                else:
                    index = min(index, max(0, len(photos) - 1))

                print(f'Media library refreshed: {len(photos)} item(s).')

        if not photos:
            draw_message(screen, [
                'Raspberry Pi Photo Frame',
                '',
                'No photos found on CUADRO.',
                str(PHOTO_DIR),
                '',
                'S = Google Drive sync',
                'R = rescan USB',
                'Esc / Q = exit',
            ])
            clock.tick(5)
            continue

        if current_surface is None or force_reload or (now - last_slide_change >= SLIDE_SECONDS):
            if current_surface is not None and not force_reload and now - last_slide_change >= SLIDE_SECONDS:
                index = (index + 1) % len(photos)

            attempts = 0
            loaded = False
            while attempts < len(photos):
                candidate = photos[index]
                try:
                    if is_video(candidate):
                        print(f'Playing [{index + 1}/{len(photos)}]: {candidate.name}')

                        # Keep pygame visibly full-screen while the video player
                        # is starting. This prevents the desktop/terminal from
                        # being intentionally exposed between slideshow items.
                        if current_surface is not None:
                            screen.blit(current_surface, (0, 0))
                        else:
                            screen.fill((0, 0, 0))

                        pygame.display.flip()
                        pygame.event.pump()

                        quit_requested = play_video(candidate)

                        if quit_requested:
                            running = False
                            loaded = True
                            break

                        # Immediately reclaim the display after VLC exits.
                        screen = pygame.display.set_mode(
                            (screen_w, screen_h),
                            pygame.FULLSCREEN | pygame.DOUBLEBUF
                        )
                        pygame.mouse.set_visible(False)
                        pygame.event.clear()

                        # Advance to the next item and load it immediately instead
                        # of dropping into the old "No readable images" branch.
                        index = (index + 1) % len(photos)
                        current_surface = None
                        current_path = None
                        last_slide_change = time.monotonic()
                        force_reload = True
                        loaded = True
                        break

                    current_surface = make_frame(candidate, screen_w, screen_h)
                    current_path = candidate
                    loaded = True
                    break

                except FileNotFoundError:
                    print('ERROR: VLC is not installed. Run: sudo apt install -y vlc')
                    index = (index + 1) % len(photos)
                    attempts += 1

                except Exception as exc:
                    print(f'Failed to load/play {candidate}: {exc}')
                    index = (index + 1) % len(photos)
                    attempts += 1

            if loaded:
                if current_surface is not None and current_path is not None:
                    print(f'Displaying [{index + 1}/{len(photos)}]: {current_path.name}')
                    last_slide_change = time.monotonic()

                # A video counts as a successfully handled media item even though
                # it does not leave a pygame image surface behind. Do NOT show
                # the old "No readable images found" message after a video.
            else:
                current_surface = None
                current_path = None
                draw_message(screen, ['No readable media found.', 'Press S to sync or R to rescan.'])

        if current_surface is not None:
            screen.blit(current_surface, (0, 0))
            pygame.display.flip()

        clock.tick(30)

    pygame.quit()
    print('Photo frame stopped.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pygame.quit()
        print('\nStopped by user.')
    except Exception as exc:
        try:
            pygame.quit()
        except Exception:
            pass
        print(f'\nFATAL ERROR: {exc}', file=sys.stderr)
        raise
