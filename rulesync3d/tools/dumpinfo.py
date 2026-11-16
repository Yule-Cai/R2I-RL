# Dump forensics: candidate vs track ranges + stall point.
import sys
import numpy as np

f = sys.argv[1]
z = np.load(f)
t = z["track"]
c = z["cand"]
print("CAND:")
print(np.asarray(c).round(2))
print("track n =", len(t))
for ax, nm in ((0, "x"), (1, "y"), (2, "z")):
    print(nm, round(float(t[:, ax].min()), 3),
          round(float(t[:, ax].max()), 3))
print("final =", np.asarray(t[-1]).round(3))
print("first =", np.asarray(t[0]).round(3))
