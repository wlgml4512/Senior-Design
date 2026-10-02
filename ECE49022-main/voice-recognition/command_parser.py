"""Phrase-to-command mapping for the standalone voice subsystem.

This parser is relevant because it translates recognized speech into the
normalized navigation and embedded actions that the rest of the robot
understands.
"""

from difflib import SequenceMatcher

class CommandParser:

    def __init__(self):
        # command dictionary
        self.commands = {
            "follow": {"subsystem": "navigation", "action": "follow"},
            "stop": {"subsystem": "embedded", "action": "emergency_stop"},
            "resume": {"subsystem": "embedded", "action": "resume"},
            "slow down": {"subsystem": "navigation", "action": "reduce_speed"},
            "speed up": {"subsystem": "navigation", "action": "increase_speed"}
        }
        self._command_tokens = {
            command: tuple(command.split()) for command in self.commands
        }

    def parse(self, text):
        """
        Match recognized text to command.
        Returns (command_dict, confidence_score) or (None, 0) if no match.
        """
        text = text.lower().strip()

        # exact match
        if text in self.commands:
            return self.commands[text], 1.0

        # fuzzy match
        words = tuple(text.split())
        if not words:
            return None, 0.0

        best_command = None
        best_confidence = 0.0

        for command, command_words in self._command_tokens.items():
            command_len = len(command_words)
            text_len = len(words)

            if command_len == 1:
                if command_words[0] in words:
                    return self.commands[command], 0.9
                score = max(SequenceMatcher(None, word, command_words[0]).ratio() for word in words)
            else:
                if command in text:
                    return self.commands[command], 0.9

                if text_len < command_len:
                    score = SequenceMatcher(None, text, command).ratio()
                else:
                    score = max(
                        SequenceMatcher(None, " ".join(words[i:i + command_len]), command).ratio()
                        for i in range(text_len - command_len + 1)
                    )

            if score > best_confidence:
                best_command = command
                best_confidence = score

        if best_confidence >= 0.75:
            return self.commands[best_command], best_confidence

        return None, 0.0
    
if __name__ == "__main__":
    parser = CommandParser()
    test_commands = ["Follow", "Stop", "Resume", "Slow down", "Speed up", "Unknown command"]
    for cmd in test_commands:
        result, confidence = parser.parse(cmd)
        print(f"Input: '{cmd}' -> Command: {result}, Confidence: {confidence}\n")
