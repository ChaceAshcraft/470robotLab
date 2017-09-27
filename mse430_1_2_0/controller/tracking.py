import cv2
import time
import numpy as np
from itertools import combinations
from operator import itemgetter


def main():
    frame = cv2.imread('test-images/led.png')
    flash = cv2.imread('test-images/bot.png')
    
    start = time.perf_counter()

    diff = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int32)
    gray = cv2.cvtColor(flash, cv2.COLOR_BGR2GRAY)

    np.subtract(diff, gray, diff)
    np.abs(diff, diff)

    _, thresh = cv2.threshold(diff.astype(np.uint8), 0, 255,
                              cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    cv2.morphologyEx(
        thresh, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), thresh,
        iterations=2)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        thresh)

    area_range = (50, 200)
    ledex = []
    for n in range(num_labels):
        if area_range[0] < stats[n,cv2.CC_STAT_AREA] < area_range[1]:
            ledex += [n]

    whatsit = []
    for pair in combinations(ledex, 2):
        whatsit += [(pair, np.linalg.norm(centroids[pair[0]] -
                                          centroids[pair[1]]))]

    whatsit.sort(key=itemgetter(1), reverse=True)
    whosit = []
    match_tolerance = 5
    for n in range(0, len(whatsit)):
        for m in range(n+1, len(whatsit)):
            if whatsit[n][1] - whatsit[m][1] < match_tolerance:
                try:
                    # import pdb; pdb.set_trace()
                    triplet = Triplet(centroids[whatsit[n][0][0]],
                                      centroids[whatsit[n][0][1]],
                                      centroids[whatsit[m][0][0]],
                                      centroids[whatsit[m][0][1]])
                    if triplet.valid_bot:
                        whosit += [triplet]
                except ValueError:
                    break

    finish = time.perf_counter()
    print('Time elapsed:', finish-start)
    
    for who in whosit:
        print(who.center)
        cv2.circle(frame, tuple(who.center.astype('int32')),
                   100, (255,0,0), 5)
        cv2.line(frame,
                 tuple(who.backc.astype('int32')),
                 tuple(who.front.astype('int32')),
                 (0,255,0), 5)

    if not whosit:
        print('Uh...')
    else:
        cv2.imshow("Frame", cv2.resize(frame, None, None, 0.5, 0.5,
                                       cv2.INTER_CUBIC))
        cv2.waitKey(0)
                

class Triplet:

    RATIO_TOLERANCE = 0.3
    
    def __init__(self, a1, b1, a2, b2):
        pair1 = (tuple(a1), tuple(b1))
        pair2 = (tuple(a2), tuple(b2))
        common = (set(pair1).intersection(pair2).pop(),)
        if not common:
            raise ValueError
        self.front = np.array(common[0])
        self.back1 = np.array(set(pair1).difference(common).pop())
        self.back2 = np.array(set(pair2).difference(common).pop())
        self.dist_long = np.mean([
            np.linalg.norm(self.front - self.back1),
            np.linalg.norm(self.front - self.back2)])
        self.backc = np.mean([self.back1, self.back2], 0)
        self.center = np.mean([self.front, self.backc], 0)
        self.dist_short = np.linalg.norm(self.back1 - self.back2)
        self.valid_bot = (abs(self.dist_long / self.dist_short - 3.0)
                          < Triplet.RATIO_TOLERANCE)


if __name__ == '__main__':
    main()
