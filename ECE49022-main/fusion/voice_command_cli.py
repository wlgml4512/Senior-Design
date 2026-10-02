"""Small CLI helper for publishing voice commands into fusion.

This script is relevant for testing because it lets developers exercise the
voice-command bridge without needing the live microphone and wake-word stack.
"""

import argparse

from interfaces.voice_interface import publish_voice_text


def _parse_args():
    parser = argparse.ArgumentParser(description="Publish a voice command into the fusion voice bridge.")
    parser.add_argument("text", help="Recognized voice text, such as 'follow', 'stop', 'resume', 'slow down', or 'speed up'.")
    parser.add_argument(
        "--source",
        default="manual_cli",
        help="Source label recorded alongside the command.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    event = publish_voice_text(args.text, source=args.source)
    print(event)
