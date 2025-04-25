from scipy.io import wavfile
from scipy.signal import lfilter
from scipy.signal.windows import hamming
import os
import glob
import numpy as np
import librosa
from spectrum import poly2lsf
from spectrum import lsf2poly


def load_audio(path):
    fs, signal = wavfile.read(path)
    print(fs)
    return signal

def pre_emphasis(signal, alpha=0.95):
    b = [1 -alpha]
    a = [1]
    return lfilter(b, a, signal)

def framming_hamming(signal, frame_len=320, hop_len=128):
    frames = []
    window = hamming(frame_len, sym=False) #probar 2

    for start in range(0, len(signal) - frame_len + 1, hop_len):
        frame = signal[start:start+frame_len] * window
        frames.append(frame)
    return np.array(frames)

def extract_lpcs(signal):
    order = 12
    return librosa.lpc(signal, order=order)

def convert_to_lpc(lsf):
    return lsf2poly(lsf)



def autocorrelacion(vec, P=12):
    rr = []
    for i in range(0, P + 1):
        r = 0.0
        for j in range(0, P - i + 1):
            r += vec[j]*vec[j+i]
        rr.append(r) 
    return rr


def distance_ItaSaito(lpc, centroid_lpc, P=12):
    rr = autocorrelacion(lpc)
    rra = autocorrelacion(centroid_lpc)
    dist = rr[0]*rra[0]
    acum = 0
    for i in range(1, P + 1):
        acum += rr[i]*rra[i]
    # (1/(sigma*sigma))* whats sigma????
    return (dist + 2*acum)




def lgbt_algorithm(lsf_vec, lpc_vec, target_centroids=16, epsilon=0.001, threshold=0.0001):
    print("length lsf vec: ", len(lsf_vec))
    centroid = np.mean(lsf_vec, axis=0)
    print(centroid.shape)
    centroids = [centroid]

    while len(centroids) < target_centroids:
        new_centroids = []
        for c in centroids:
            new_centroids.append(c * (1 + epsilon))
            new_centroids.append(c * (1 - epsilon))
        centroids = new_centroids
        # print("len centroids: ", len(centroids))
        # print("centroids: ", centroids)

        
        last_total_distorsion = 200000000
        converged = False
        while not converged:
            clusters = [[] for _ in centroids]
            total_distorsion = 0
            

            for i, vec in enumerate(lpc_vec):
                distances = [distance_ItaSaito(vec, convert_to_lpc(c)) for c in centroids]
                idx_minor_distance = np.argmin(distances)
                clusters[idx_minor_distance].append(lsf_vec[i])
                total_distorsion += distances[idx_minor_distance] 
            # print(total_distorsion)
            print("clusters: ", len(clusters))

            diff = abs(total_distorsion - last_total_distorsion)
            if diff < threshold:
                converged = True
            else:
                new_centroids = []
                for cluster in clusters:
                    if cluster:
                        new_centroids.append(np.mean(cluster, axis=0))
                centroids = new_centroids
                print("centroid: ", centroids)

                last_total_distorsion = total_distorsion

            # print('difference: ', diff)
            # print('centroids: ', centroids)


            
    
    return centroids




def convert_to_lsf(word_lpcs):
    lsf_coeff_signal = []
    for lpc in word_lpcs:
        lsf_values = poly2lsf(lpc)
        lsf_coeff_signal.append(lsf_values)
    return lsf_coeff_signal

def signal_treatment(path, lpcs_list):
    signal = load_audio(path)
    signal = pre_emphasis(signal)
    frames = framming_hamming(signal)
    for window in frames:
        lpcs = extract_lpcs(window)
        lpcs_list.append(lpcs)
    return frames, lpcs_list

def main():
    word_frames = []
    word_lpcs = []
    lsf_coeff_signal = []

    root = os.getcwd()
    audioDir = os.path.join(root, 'audioDir')
    audioPathList = glob.glob(os.path.join(audioDir, 'empieza_*.wav'))
    for path in audioPathList:
        frames, word_lpcs = signal_treatment(path, word_lpcs)
        word_frames.append(frames)

    #Make one single list
    # word_frames.extend()

    lsf_coeff_signal = convert_to_lsf(word_lpcs=word_lpcs)
    codebook = lgbt_algorithm(lsf_coeff_signal, word_lpcs)

    print(codebook)





if __name__ == '__main__':
    main()