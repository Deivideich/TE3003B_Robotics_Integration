import time

def mockable(return_value=None, delay=0, mock=False):
    """
    Decorator to return mock values instead of performing
    the function.
    Args:
        return_value: Value to return if mock_data is True
        delay: Delay in seconds before returning the value
    """

    def decorator(func):
        def wrapper(self, *args, **kwargs):
            if getattr(self, "mock_data", False) or mock:
                if delay > 0:
                    time.sleep(delay)
                self.node.get_logger().info(f"{func.__name__}. Value: {return_value}")
                return return_value

            return func(self, *args, **kwargs)

        wrapper.__name__ = func.__name__

        return wrapper

    return decorator