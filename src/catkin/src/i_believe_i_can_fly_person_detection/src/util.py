#!/usr/bin/env python


def is_empty_or_none(array):
    """
    Checks if the given array is empty.
    :param array: array to check
    :return: True if array is None or empty, otherwise False.
    """
    return array is None or len(array) == 0
