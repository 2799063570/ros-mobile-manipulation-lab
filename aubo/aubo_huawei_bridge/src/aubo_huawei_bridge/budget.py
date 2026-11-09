"""Persistently bound this bridge's outgoing 4KB units in every rolling 24 hours."""
import sqlite3
import threading
import time


class PublishBudget:
    def __init__(self, path, limit=4000, reply_reserve=100, clock=time.time):
        if type(limit) is not int or not 200 <= limit <= 8000 or not 0 <= reply_reserve < limit:
            raise ValueError('invalid_message_budget')
        self.limit, self.reserve, self.clock = limit, reply_reserve, clock
        self.lock = threading.Lock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS publishes (stamp REAL NOT NULL, units INTEGER NOT NULL)')
        self.db.commit()

    def take(self, units=1, reply=False):
        if type(units) is not int or units <= 0:
            raise ValueError('invalid_units')
        with self.lock, self.db:
            now = self.clock()
            self.db.execute('DELETE FROM publishes WHERE stamp <= ?', (now - 86400,))
            used = self.db.execute('SELECT COALESCE(SUM(units), 0) FROM publishes').fetchone()[0]
            ceiling = self.limit if reply else self.limit - self.reserve
            if used + units > ceiling:
                return False
            self.db.execute('INSERT INTO publishes VALUES (?, ?)', (now, units))
            return True

    def used(self):
        with self.lock:
            return self.db.execute('SELECT COALESCE(SUM(units), 0) FROM publishes WHERE stamp > ?',
                                   (self.clock() - 86400,)).fetchone()[0]
