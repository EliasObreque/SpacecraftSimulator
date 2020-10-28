import subprocess
from pathlib import Path
import requests
from tqdm import tqdm


if __name__ == '__main__':

    # Install error state kalman filter
    kalman_repo = 'https://github.com/gdiazh/error_state_kalman_filter'
    kalman_branch = 'feat-satsimulator-integration'
    from_folder = Path('Components/Logic/Estimation')
    repo_folder = Path(from_folder, 'error_state_kalman_filter', '.git')
    if not repo_folder.exists():
        print('Kalman Filter repo not found.')
        print('Cloning repo ', kalman_repo, '...')
        res = subprocess.run(['git', 'clone', '-b', kalman_branch, kalman_repo], cwd=str(from_folder))
    else:
        print('Kalman Filter repo found.')
        print('Pulling repo...')
        res = subprocess.run(['git', 'pull', 'origin', kalman_branch], cwd=str(Path(from_folder, 'error_state_kalman_filter')))

    # Install generic kernels
    from_folder = Path('Dynamics/CelestialBody/cspice/generic_kernels/spk/planets')
    kernel_file = Path(from_folder, 'de430.bsp')
    if not kernel_file.exists():
        print('File ', kernel_file, ' not found.')
        kernel_url = 'https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp'

        with requests.get(kernel_url, stream=True) as r:
            r.raise_for_status()
            total_size = int(r.headers.get('content-length', 0))
            block_size = 1024*1024
            print('Downloading file de430.bsp...')
            t = tqdm(total=total_size, unit='iB', unit_scale=True)
            with open(kernel_file, 'wb') as f:
                for chunk in r.iter_content(chunk_size=block_size):
                    t.update(len(chunk))
                    f.write(chunk)
    else:
        print('File ', kernel_file, ' found.')
