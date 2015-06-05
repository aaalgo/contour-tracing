#ifndef WDONG_CONTOUR_TRACING
#define WDONG_CONTOUR_TRACING

#include <iostream>
#include <boost/assert.hpp>
#include <opencv2/core/core.hpp>

namespace cv {

    namespace contour_tracing_impl {

        //using namespace std;

        // moor neighbors
        static int rel_x[] = {-1, 0, 1, 1, 1, 0, -1, -1};
        static int rel_y[] = {-1, -1, -1, 0, 1, 1, 1, 0};
        static int rel_map[] = {5, 7, 7, 1, 1, 3, 3, 5};

        template <typename TYPE>
        bool is_in (Mat const &image, TYPE refv, Point const &point) {
            if (point.x < 0) return false;
            if (point.y < 0) return false;
            if (point.x >= image.cols) return false;
            if (point.y >= image.rows) return false;
            return image.at<TYPE>(point) == refv;
        }

        // Trace outer countour
        // The region is black, contour is white.
        template <typename TYPE>
        void MoorContourTracingImpl (Mat const &image, Point const &ref, vector<Point> *contour) {
            TYPE refv = image.at<TYPE>(ref);

            Point black = ref;
            Point start = black;
            // move out of region by increasing x
            for (;;) {
                ++start.x;
                if (!is_in(image, refv, start)) break;
                black = start;
            }
            //          |2
            //  region  |3  <-- now we are at location 3
            //   -------+4
            contour->clear();
            contour->push_back(start);
            unsigned rel = 3;
            for (;;) {
                // we are circumvent black, and at relative location rel of black
                for (;;) {
                    rel = (rel + 1) % 8;
                    Point next;
                    next.x = black.x + rel_x[rel];
                    next.y = black.y + rel_y[rel];
                    if (is_in(image, refv, next)) {
                        //cerr << black.x << ' ' << black.y << " = "  << next.x << ' ' << next.y << endl;
                        black = next;
                        rel = rel_map[rel];
                        break;
                    }
                    //cerr << black.x << ' ' << black.y << " ! "  << next.x << ' ' << next.y << endl;
                    if (next == start) return;
                    bool good = false;
                    for (unsigned j = 0; j < 8; ++j) {
                        Point xx = next;
                        xx.x += rel_x[j];
                        xx.y += rel_y[j];
                        if (is_in(image, refv, xx)) {
                            good = true;
                            break;
                        }
                    }
                    BOOST_VERIFY(good);
                    contour->push_back(next);
                }
            }
        }
    }

    void MoorContourTracingImpl (Mat const &image, Point const &ref, vector<Point> *contour) {
        using namespace contour_tracing_impl;
        BOOST_VERIFY(image.channels() == 1);
        switch (image.depth()) {
            case CV_32S:
                MoorContourTracingImpl<int32_t>(image, ref, contour);
                break;
            default:
                BOOST_VERIFY(0);
        }
    }
}

#endif
