import h5py

with h5py.File('data.h5', 'r') as f:
    print("Datasets:", list(f.keys()))
    for name in f:
        dset = f[name]
        data = dset[()]
        print(f"'{name}' shape={data.shape}, dtype={data.dtype}")
        print("Sample data:", data.flat[:5])
