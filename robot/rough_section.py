class RoughSection:
    """Future rough-terrain state; reserved for obstacle/corridor analysis."""

    name = "rough_section"

    @staticmethod
    def analyze(_hsv):
        return {"ready": False, "reason": "obstacle perception not implemented"}

    @staticmethod
    def status_lines(result):
        return ["rough section is not active", "planner=" + result["reason"]]
