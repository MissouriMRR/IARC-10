import numpy as np
import numpy.typing as npt
from skimage.feature import local_binary_pattern
from scipy.spatial.distance import euclidean

def get_lbp_hist(image):
    r: int = 2
    p: int = 16*r
    #find the lbp first
    lbp = local_binary_pattern(image,P=p,R=r,method='uniform')

    # this can be tweaked to +2, +1, or +0
    n_bins = 8 + 2

    # the actual function
    hist, _ = np.histogram(lbp.ravel(),bins= n_bins, range= (0,n_bins),density= True)

    return hist

def compare_textures(hist1,hist2, method = 'euclidean'):
    # these are just a bunch of different ways of comparing histograms
    # the best options for our use case are euclidean, bhattacharyya, or intersection
    if(method == 'euclidean'):
        return 100*(euclidean(hist1,hist2))
    if(method == 'correlation'):
        return np.corrcoef(hist1,hist2)[0,1]
    if(method == 'bhattacharyya'):
        return 100*(-np.log(np.sum(np.sqrt(hist1 * hist2)) + 1e-10))
    if(method == 'intersection'):
        return 100*(1 - np.sum(np.minimum(hist1, hist2)))
    if(method == 'chi_squared'):
        return 0.5 * np.sum((hist1 - hist2)**2 / (hist1 + hist2 + 1e-10))