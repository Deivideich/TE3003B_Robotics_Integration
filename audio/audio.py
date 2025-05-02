from scipy.io import wavfile
from scipy.signal import lfilter
from scipy.signal.windows import hamming
import os
import glob
import numpy as np
import librosa
from spectrum import poly2lsf
from spectrum import lsf2poly
from sklearn.metrics import confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import re

def load_audio(path):
    fs, signal = wavfile.read(path)
    return signal

# Obtain signal with high filter filtering
def pre_emphasis(signal, alpha=0.95):
    b = [1 -alpha]
    a = [1]
    return lfilter(b, a, signal)

# Frame a signal and obtain window with hamming treatment
def framming_hamming(signal, frame_len=320, hop_len=128):
    frames = []
    window = hamming(frame_len, sym=False)

    for start in range(0, len(signal) - frame_len + 1, hop_len):
        frame = signal[start:start+frame_len] * window
        frames.append(frame)
    return np.array(frames)

# Extract LPCs from window. Use Burg method.
def extract_lpcs(signal):
    order = 12
    return librosa.lpc(signal, order=order)

def convert_to_lpc(lsf):
    return lsf2poly(lsf)

def convert_to_lsf(word_lpcs):
    lsf_coeff_signal = []
    for lpc in word_lpcs:
        lsf_values = poly2lsf(lpc)
        lsf_coeff_signal.append(lsf_values)
    return lsf_coeff_signal

# Autocorrelation LPC algorithm
def autocorrelacion(vec, P=12):
    rr = []
    N = len(vec)
    for i in range(0, P + 1):
        r = 0.0
        for j in range(0, N - i):
            r += vec[j]*vec[j+i]
        r = r/N
        rr.append(r)
    return rr


# short autocorrelation LPC algorithm
def autocorrelacion_corta(vec, P=12):
    rr = []
    for i in range(0, P + 1):
        r = 0.0
        for j in range(0, P - i + 1):
            r += vec[j]*vec[j+i]
        rr.append(r) 
    return rr

# Itakura-Saito distance
def distance_ItaSaito(frame, centroid_lpc, P=12):
    rr = autocorrelacion(frame)
    rra = autocorrelacion_corta(centroid_lpc)
    dist = rr[0]*rra[0]
    acum = 0
    for i in range(1, P + 1):
        acum += rr[i]*rra[i]
    return (dist + 2*acum)

def distance_euclidean(lsf1, lsf2):
    return np.linalg.norm(lsf1 - lsf2)


# High filter signal, make windows, use Hamming window and obtain LPCs of each window.
def signal_treatment(path, lpcs_list, word_frames):
    signal = load_audio(path)
    signal = pre_emphasis(signal)
    frames = framming_hamming(signal)
    
    for window in frames:
        lpcs = extract_lpcs(window)
        word_frames.append(window)
        lpcs_list.append(lpcs)
    return word_frames, lpcs_list

def lbg_algorithm(lsf_vec, word_frames, target_centroids=16, epsilon=0.005, threshold=0.1, flagIta = True):
    print("length lsf vec: ", len(lsf_vec))
    centroid = np.mean(lsf_vec, axis=0)
    print(centroid.shape)
    centroids = [centroid]

    while len(centroids) < target_centroids:
        new_centroids = []
        last_total_distorsion = 200000000
        converged = False

        # Obtain double quantity of centroids from actual quantity of centroids, using epsilon as difference
        for c in centroids:
            new_centroids.append(c * (1 + epsilon))
            new_centroids.append(c * (1 - epsilon))
        centroids = new_centroids
          
        while not converged:
            # Obtain number of clusters as number of centroids
            clusters = [[] for _ in centroids]
            total_distorsion = 0
            print("clusters: ", len(clusters))
            
            # For each word frame, add frame to nearest cluster and accumulate minor distance.
            for i, vec in enumerate(word_frames):
                if flagIta:
                    distances = [distance_ItaSaito(vec, convert_to_lpc(c)) for c in centroids] #Using LPCs, obtain distance of one word frame to each centroid
                else:
                    distances = [distance_euclidean(lsf_vec[i], c) for c in centroids] # Using euclidean distance, obtain distance of one word frame LSF coefficient to each centroid
                idx_minor_distance = np.argmin(distances)
                clusters[idx_minor_distance].append(lsf_vec[i])
                total_distorsion += distances[idx_minor_distance] 

            # If difference between new and last total distorsion is less than threshold, centroids are in their optimal position
            diff = abs(total_distorsion - last_total_distorsion)
            if diff < threshold:
                converged = True
            # If not, obtain new centroids using mean of all LSF coefficients that are in cluster.
            else:
                new_centroids = []
                for cluster in clusters:
                    if cluster:
                        new_centroids.append(np.mean(cluster, axis=0))
                centroids = new_centroids
                last_total_distorsion = total_distorsion

    return centroids

#For one folder full of audios, obtain codebook of one word
def training(folder_path, folder_name):
    word_frames = []
    word_lpcs = []
    lsf_coeff_signal = []

    audioPathList = glob.glob(os.path.join(folder_path, f'{folder_name}*.wav'))
    #Obtain word frames and LPCs for each audio, and put them together in one list.
    for path in audioPathList:
        word_frames, word_lpcs = signal_treatment(path, word_lpcs, word_frames)

    # Obtain LSF coefficients of all LPCs for one word.
    lsf_coeff_signal = convert_to_lsf(word_lpcs=word_lpcs)
    #Obtain codebook (centroids) using LBG algorithm
    codebook = lbg_algorithm(lsf_coeff_signal, word_frames=word_frames)

    print("final len codebook: ", len(codebook))

    # Sabe centroids in .npz
    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, f'{folder_name}.npz')
    np.savez(paramPath, name=folder_name, codebook=codebook)

def test(testPath, codebooks, codebooks_name, flagIta=False):
    word_frames = []
    word_lpcs = []
    lsf_coeff_signal = []
    min_distance_global = np.inf

    # High filter signal, make windows, use Hamming window and obtain LPCs of each window.
    word_frames, word_lpcs = signal_treatment(testPath, lpcs_list=word_lpcs, word_frames=word_frames)
    # Obtain LSF from each LPC coefficient.
    lsf_coeff_signal = convert_to_lsf(word_lpcs=word_lpcs)

    # For one signal, compare with all centroids from all codebooks using euclidean or Itakura Saito distance.
    # The codebook who has minor total distance between all centroids from all window frames, is the codebook that gets chosen.
    for i, codebook in enumerate(codebooks):
        total_dist = 0
        for j, frame in enumerate(word_frames):
            if flagIta:
                # Compare each window frame with LPC centroid
                dist = [distance_ItaSaito(frame, convert_to_lpc(c)) for c in codebook]
            else: 
                # Compare window LSF coefficients with LSF centroid
                dist = [distance_euclidean(lsf_coeff_signal[j], c) for c in codebook] 
            total_dist += np.min(dist)
        if total_dist < min_distance_global:
            min_distance_global = total_dist
            idx_nearest_codebook = i

    return codebooks_name[idx_nearest_codebook]

def main(train=False):
    root = os.getcwd()
    if train:
        
        audioDir = os.path.join(root, 'audioDir')
        folder_names = []
        folder_path = []
        for f in glob.glob(os.path.join(audioDir, '*')):
            if os.path.isdir(f):
                folder_names.append(os.path.basename(f))
                folder_path.append(os.path.join(audioDir, f))
        
        for i, folder in enumerate(folder_path):
            name_folder = folder_names[i]
            npz_path = os.path.join(root, f'{name_folder}.npz')
            if not os.path.exists(npz_path):
                training(folder, name_folder)
            else:
                print(f'Ya existe {name_folder}.npz, omitiendo')
    else:
        codebooks = []
        codebooks_name = []
        codebooksPath = glob.glob(os.path.join(root, '*.npz'))

        #Extract codebook npz info
        for codebook in codebooksPath:
            data = np.load(codebook)
            codebooks_name.append(data['name'])
            codebooks.append(data['codebook'])

        y_true = []
        y_pred = []
        pruebaFolder = os.path.join(root, 'pruebas')
        testPathList = glob.glob(os.path.join(pruebaFolder, '*.wav'))
        # For each audio signal, test all codebooks and see which codebook is nearest to signal.
        for testPath in testPathList:
            testName = os.path.basename(testPath)
            y_true.append(re.split(r'[-_]', testName)[0])
            result = test(testPath, codebooks, codebooks_name)
            y_pred.append(result)

        print('y_true: ', y_true)
        print('y_pred: ', y_pred)

        cm = confusion_matrix(y_true, y_pred)

        sns.heatmap(cm, annot=True, fmt="d", cmap="Blues")
        plt.xlabel("Predicted")
        plt.ylabel("True")
        plt.title("Confusion Matrix")
        plt.show()




if __name__ == '__main__':
    main()