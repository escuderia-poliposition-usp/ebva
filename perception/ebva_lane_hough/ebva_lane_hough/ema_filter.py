class EMAFilter:
    """
    Exponential Moving Average filter.

    y[k] = alpha*x[k] + (1-alpha)*y[k-1]
    """

    def __init__(self, alpha=0.35):
        self.alpha = float(alpha)
        self.value = None

    def reset(self):
        self.value = None

    def update(self, value):
        value = float(value)

        if self.value is None:
            self.value = value
        else:
            self.value = (
                self.alpha * value
                + (1.0 - self.alpha) * self.value
            )

        return self.value
