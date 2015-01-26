__author__ = 'manuelviermetz'

import time
import numpy as np


class speedtest:
    def __init__(self):
        self.last_start = 0
        self.result_dim = 10
        self.results = np.zeros([self.result_dim])
        self.call_rate = np.zeros([self.result_dim])
        self.result_pos = 0
        self.last_printout = 0

    def start(self):
        self.call_rate[self.result_pos] = 1. / (time.time() - self.last_start)
        self.last_start = time.time()

    def stop(self):
        self.results[self.result_pos] = time.time() - self.last_start
        self.result_pos += 1
        if self.result_pos == self.result_dim:
            self.result_pos = 0

    def result_time(self):
        return np.mean(self.results), np.std(self.results)

    def result_rate(self):
        return np.mean(self.call_rate)

    def print_result(self, rate=1., text='speed test result time:'):
        if time.time() > self.last_printout:
            print text, self.result_time(), 'sec', self.result_rate(), 'Hz'
            self.last_printout = time.time() + 1. / rate
