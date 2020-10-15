// Imagine++ project
// Project:  Panorama
// Base author:   Pascal Monasse
// Project author: Timothée Darcet
// Date:     2013/10/08


#include <Imagine/Graphics.h>
#include <Imagine/Images.h>
#include <Imagine/LinAlg.h>
#include <vector>
#include <sstream>
using namespace Imagine;
using namespace std;

// Comment / Uncomment these lines in order to change the pixel transfer mode
// PULL: Pull each pixel in the reconstructed panorama from the source images
// PUSH: Push each pixel in the source images to the panorama.
// PUSH produces artefacts, as it does not guarantee each pixel in the panorama will be filled.
#define PULL
// #define PUSH

// Record clicks in two images, until right button click
void getClicks(Window w1, Window w2,
               vector<IntPoint2>& pts1, vector<IntPoint2>& pts2) {
    int xc, yc;
    Window wc;
    int subwc;
    // Loop while the user has not right clicked yet
    while (anyGetMouse(xc, yc, wc, subwc) != 3) {
        IntPoint2 ip2 = IntPoint2(xc, yc);
        setActiveWindow(wc, subwc);
        // Draw a circle to show the clicked point
        drawCircle(ip2, 10, Color(50, 50, 200), 3);
        if (wc == w1) {
            pts1.push_back(ip2);
            cout << "clicked w1 at " << xc << ", " << yc << endl;
        }
        else if (wc == w2) {
            pts2.push_back(ip2);
            cout << "clicked w2 at " << xc << ", " << yc << endl;
        }
    }
    cout << "Right clicked" << endl;
    cout << "=============" << endl;

    // Uncomment this to add some prerecorded suitable points. Useful for debugging.
    pts2.push_back({222, 390});
    pts2.push_back({247, 243});
    pts2.push_back({75, 146});
    pts2.push_back({75, 376});
    pts1.push_back({679, 395});
    pts1.push_back({705, 247});
    pts1.push_back({531, 147});
    pts1.push_back({533, 379});
}

// Return homography compatible with point matches
Matrix<float> getHomography(const vector<IntPoint2>& pts1,
                            const vector<IntPoint2>& pts2) {
    size_t n = min(pts1.size(), pts2.size());
    if(n<4) {
        cout << "Not enough correspondences: " << n << endl;
        return Matrix<float>::Identity(3);
    }
    Matrix<double> A(2*n,8);
    Vector<double> B(2*n);
    int i;
    // Fill in the matrix A and the vector B before solving
    for(i = 0; i < n; i++){
        double xi = (double)pts1.at(i)[0];
        double yi = (double)pts1.at(i)[1];
        double xi2 = (double)pts2.at(i)[0];
        double yi2 = (double)pts2.at(i)[1];
        A.setRow(2*i, Imagine::Array<double>({xi, yi, 1, 0, 0, 0, -xi2*xi, -xi2*yi}));
        A.setRow(2*i+1, Imagine::Array<double>({0, 0, 0, xi, yi, 1, -yi2*xi, -yi2*yi}));
        B[2*i] = xi2;
        B[2*i+1] = yi2;
    }

    // If there are four points : solve A*x=B
    // If there are more : minimize |A*x-B|
    B = linSolve(A, B);
    Matrix<float> H(3, 3);
    // Unwrap B to the matrix H
    H(0,0)=B[0]; H(0,1)=B[1]; H(0,2)=B[2];
    H(1,0)=B[3]; H(1,1)=B[4]; H(1,2)=B[5];
    H(2,0)=B[6]; H(2,1)=B[7]; H(2,2)=1;

    // Sanity check
    for(size_t i=0; i<n; i++) {
        float v1[]={(float)pts1[i].x(), (float)pts1[i].y(), 1.0f};
        float v2[]={(float)pts2[i].x(), (float)pts2[i].y(), 1.0f};
        Vector<float> x1(v1,3);
        Vector<float> x2(v2,3);
        x1 = H*x1;
        cout << x1[1]*x2[2]-x1[2]*x2[1] << ' '
             << x1[2]*x2[0]-x1[0]*x2[2] << ' '
             << x1[0]*x2[1]-x1[1]*x2[0] << endl;
    }
    return H;
}

// Grow rectangle of corners (x0,y0) and (x1,y1) to include (x,y)
void growTo(float& x0, float& y0, float& x1, float& y1, float x, float y) {
    if(x<x0) x0=x;
    if(x>x1) x1=x;
    if(y<y0) y0=y;
    if(y>y1) y1=y;    
}

// Paint color c at coords (x, y) in image I
// If the pixel that was there before was white, just replace it.
// If there was already a non-white color, blend with it
void doThePaint(Image<Color, 2>& I, const int x, const int y, Color c) {
    if (I(x, y) == WHITE) {
        I(x, y) = c;
    } 
    else {
        I(x, y) = c/2. + I(x, y)/2.;
    }
}

// Try to remove cToRemove from the image I
// Replaces the pixels that have this color with a blend of neighbouring pixels
void inPaintColor(Image<Color, 2>& I, const Color cToRemove) {
    for (int xi=0; xi<I.width(); xi++) {
        for (int yi=0; yi<I.height(); yi++) {
            if (I(xi, yi) == cToRemove) {
                cout << "Found " << cToRemove << " at " << xi << " " << yi << endl;
                Color c = BLACK;
                int denom = 0;
                // Mix the colors from the neighouring pixels
                for (int a=-1; a<=1; a++) {
                    for (int b=-1; b<=1; b++) {
                        if (xi+a >= 0 && xi+a < I.width() && yi+b>=0 && yi+b < I.height() && I(xi+a, yi+b) != cToRemove) {
                            c += I(xi+a, yi+b);
                            denom++;
                        }
                    }
                }
                if (denom > 0)
                    I(xi, yi) = c/float(denom);
            }
        }
    }
}

// Unused, does not work well at all
// Try to remove artifacts by detecting points which are very different from their surrounding points, then inpainting them
void inPaintWeirdos(Image<Color, 2>& I) {
    for (int xi=0; xi<I.width(); xi++) {
        for (int yi=0; yi<I.height(); yi++) {
            // Calculate mean of points around
            Color c = BLACK;
            int denom = 0;
            for (int a=-1; a<=1; a++) {
                for (int b=-1; b<=1; b++) {
                    if (xi+a >= 0 && xi+a < I.width() && yi+b>=0 && yi+b < I.height() && (a!=0 || b!=0)) {
                        c += I(xi+a, yi+b);
                        denom++;
                    }
                }
            }
            // If the mean around is too far from the point value, inPaint with the point instead
            float mc = (c.x() + c.y() + c.z()) / 3.;
            float mI = (I(xi, yi).x() + I(xi, yi).y() + I(xi, yi).z()) / 3.;
            if (denom > 0 && abs(mc - mI) > 100)
                I(xi, yi) = c/denom;
        }
    }
}

// Simple int comparison helper function
bool in(int a, int b, int c) {
    return a <= b && b < c;
}

// Panorama construction
void panorama(const Image<Color,2>& I1, const Image<Color,2>& I2,
              Matrix<float> H) {
    // Find the bounding box of the panorama image
    Vector<float> v(3);
    float x0=0, y0=0, x1=I2.width(), y1=I2.height();

    v[0]=0; v[1]=0; v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    v[0]=I1.width(); v[1]=0; v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    v[0]=I1.width(); v[1]=I1.height(); v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    v[0]=0; v[1]=I1.height(); v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    cout << "x0 x1 y0 y1=" << x0 << ' ' << x1 << ' ' << y0 << ' ' << y1<<endl;

    // Create image
    int w = int(x1-x0);
    int h = int(y1-y0);
    Image<Color> I(w, h);
    setActiveWindow(openWindow(I.width(), I.height()) );
    I.fill(WHITE);

    // Transfer pixels
    #ifdef PUSH
    // Transfer from I2 with a simple translation 
    for (int xi=0; xi<I2.width(); xi++) {
        for (int yi=0; yi<I2.height(); yi++) {
            int x = xi - x0;
            int y = yi - y0;
            doThePaint(I, x, y, I2(xi, yi));
        }
    }
    // Transfer from I1 via the homography multiplication and a translation
    for (int xi=0; xi<I1.width(); xi++) {
        for (int yi=0; yi<I1.height(); yi++) {
            v[0]=xi; v[1]=yi; v[2]=1;
            v=H*v;
            v/=v[2];
            int x = v[0] - x0;
            int y = v[1] - y0;
            doThePaint(I, x, y, I1(xi, yi));
        }
    }
    // Try to correct the artifacts
    // Works a little bit. Not much.
    inPaintColor(I, WHITE);
    inPaintColor(I, BLACK);
    inPaintColor(I, BLACK);
    inPaintColor(I, BLACK);
    // inPaintWeirdos(I); // Does not work at all
    #endif
    #ifdef PULL
    // Iterate on the panorama pixels
    for (int xi=0; xi<I.width(); xi++) {
        for (int yi=0; yi<I.height(); yi++) {
            // Calculate coordinates in I2
            int xi2 = xi + x0;
            int yi2 = yi + y0;
            // Calculate coordinates in I1
            Matrix<float> Hi = inverse(H);
            v[0] = xi2; v[1] = yi2; v[2] = 1;
            v = Hi * v;
            v /= v[2];
            int xi1 = v[0];
            int yi1 = v[1];
            // Check if the calculated coordinates are in the picture
            bool okay1 = in(0, xi1, I1.width()) && in(0, yi1, I1.height());
            bool okay2 = in(0, xi2, I2.width()) && in(0, yi2, I2.height());
            I(xi, yi) = Color(0, 0, 0);
            // Use I1, I2, both, or none, depending on the validity of the coordinates
            if (okay1) {
                I(xi, yi) += I1(xi1, yi1) / 2.;
            }
            if (okay2) {
                I(xi, yi) += I2(xi2, yi2) / 2.;
            }
            if (okay1 != okay2) {
                I(xi, yi) *= 2;
            }
        }
    }
    #endif    
    display(I,0,0);
}

// Main function
int main(int argc, char* argv[]) {
    const char* s1 = argc>1? argv[1]: srcPath("image0006.jpg");
    const char* s2 = argc>2? argv[2]: srcPath("image0007.jpg");

    // Load and display images
    Image<Color> I1, I2;
    if( ! load(I1, s1) ||
        ! load(I2, s2) ) {
        cerr<< "Unable to load the images" << endl;
        return 1;
    }
    Window w1 = openWindow(I1.width(), I1.height(), s1);
    display(I1,0,0);
    Window w2 = openWindow(I2.width(), I2.height(), s2);
    setActiveWindow(w2);
    display(I2,0,0);

    // Get user's clicks in images
    vector<IntPoint2> pts1, pts2;
    getClicks(w1, w2, pts1, pts2);

    vector<IntPoint2>::const_iterator it;
    cout << "pts1="<<endl;
    for(it=pts1.begin(); it != pts1.end(); it++)
        cout << *it << endl;
    cout << "pts2="<<endl;
    for(it=pts2.begin(); it != pts2.end(); it++)
        cout << *it << endl;

    // Compute homography
    Matrix<float> H = getHomography(pts1, pts2);
    cout << "H=" << H/H(2,2);

    // Apply homography
    panorama(I1, I2, H);

    endGraphics();
    return 0;
}
