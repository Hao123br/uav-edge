from sklearn.cluster import KMeans
import numpy as np

# read points from file
# write centroids to another file

def main():

    with open("unserved.txt") as f:
        lines = [[float(i.split()[1]), float(i.split()[2])] for i in f.readlines()]
        if (len(lines) == 0):
            return
        lines = np.array(lines)

        try:
            kmeans = KMeans(n_clusters=10, random_state=0).fit(lines)

            centroids = kmeans.cluster_centers_

            np.savetxt("centroids.txt", centroids, delimiter=" ")
        except Exception:
            pass
if __name__ == "__main__":
    main()
