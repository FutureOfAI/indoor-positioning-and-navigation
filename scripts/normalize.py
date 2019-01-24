import numpy as np
import librosa

x = np.ones(3)
y = np.ones(3)*2
z = np.ones(3)*3
d = np.array([x,y,z])


f = librosa.util.normalize(d, norm=2)

print f