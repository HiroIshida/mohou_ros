import pickle

filename = './rgb_image.pkl'
with open(filename, 'rb') as f:
    img = pickle.load(f)
print(img.data)
