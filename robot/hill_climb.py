class HillClimb:
    """Future hill-climb state; reserved for ramp alignment and steady drive."""

    name = "hill_climb"

    @staticmethod
    def analyze(_hsv):
        return {"ready": False, "reason": "ramp perception not implemented"}

    @staticmethod
    def status_lines(result):
        return ["hill climb is not active", "ramp controller=" + result["reason"]]
