import os

def file_basenames(dir):
	if not os.path.exists(dir):
		os.makedirs(dir)

	filenames = os.listdir(dir)
	basenames = [os.path.splitext(filename)[0] for filename in filenames]
	return sorted(basenames)
