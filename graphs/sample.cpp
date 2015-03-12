#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>

#include "gc_min.hpp"

void onMouse( int event, int x, int y, int, void* s) ;

struct sel {
    cv::Mat im;
    cv::Rect w;
    cv::Point origin;
    bool state;
    int wid, hei;
    bool track;
};

sel getWin(cv::VideoCapture &cap);

int main( int argc, char** argv ) {

    cv::VideoCapture cap(0);
    sel win = getWin(cap);

    cv::Mat img = win.im.clone();
    cv::Rect hand_win = win.w,
        outter_win = cv::Rect(0, 0, win.wid, win.hei)&
            (cv::Rect(win.w.x-25,win.w.y-25,win.w.width+50,win.w.height+50));

    cv::rectangle(img, hand_win, cv::Scalar(0,255,0));
    cv::rectangle(img, outter_win, cv::Scalar(0,255,255));
    cv::imshow("IM",img);

    cv::ml::EM::Params params_em(3, cv::ml::EM::COV_MAT_DIAGONAL);
    cv::Ptr<cv::ml::EM> emBG = cv::ml::EM::create(params_em),
            emFG = cv::ml::EM::create(params_em);

    int num_elemsFG = hand_win.width * hand_win.height;
    int num_elemsBG = outter_win.width * outter_win.height - num_elemsFG;
    cv::Mat trainFG(num_elemsFG, 3, CV_64FC1);
    cv::Mat trainBG(num_elemsBG, 3, CV_64FC1);
    cv::Mat maskFG( img.size(), CV_8UC1, cv::Scalar(0) );
    cv::Mat maskBG( img.size(), CV_8UC1, cv::Scalar(0) );
    cv::rectangle(maskFG, hand_win, cv::Scalar(255), cv::FILLED);
    cv::rectangle(maskBG, outter_win, cv::Scalar(255), cv::FILLED);
    cv::rectangle(maskBG, hand_win, cv::Scalar(0), cv::FILLED);

    cv::Point p;
    int bg_row=0, fg_row=0;

    for( p.y = 0; p.y < img.rows; p.y++ ) {
        for( p.x = 0; p.x < img.cols; p.x++) {
            cv::Vec3b col = img.at<cv::Vec3b>(p);

            if (maskBG.at<uchar>(p)!=0) {
                trainBG.at<double>(bg_row,0) = (double)col[0];
                trainBG.at<double>(bg_row,1) = (double)col[1];
                trainBG.at<double>(bg_row,2) = (double)col[2];
                bg_row++;
            }

            if (maskFG.at<uchar>(p)!=0) {
                trainFG.at<double>(fg_row,0) = (double)col[0];
                trainFG.at<double>(fg_row,1) = (double)col[1];
                trainFG.at<double>(fg_row,2) = (double)col[2];
                fg_row++;
            }
        }
    }

    emBG = emBG->train(trainBG);
    emFG = emFG->train(trainFG);

    //std::cout<<"BG="<<stb<<std::endl;
    //std::cout<<"FG="<<stf<<std::endl;

    int vtxCount = img.cols*img.rows,
   edgeCount = 2*(4*img.cols*img.rows - 3*(img.cols + img.rows) + 2);

    GC_Solver<double> graph;
    graph.create(vtxCount, edgeCount);

    cv::Mat eimg = img(hand_win).clone();
    for( p.y = 0; p.y < eimg.rows; p.y++ ) {
        for( p.x = 0; p.x < eimg.cols; p.x++) {
            // add node
            int vtxIdx = graph.addVtx();
            cv::Mat color = cv::Mat(1,3,CV_64FC1);
            cv::Vec3d col;
            col[0] = color.at<double>(0,0) = eimg.at<cv::Vec3b>(p)[0];
            col[1] = color.at<double>(0,1) = eimg.at<cv::Vec3b>(p)[1];
            col[2] = color.at<double>(0,2) = eimg.at<cv::Vec3b>(p)[2];

            // set t-weights
            double fromSource, toSink;
            cv::Mat posterior;
            fromSource = emBG->predict2(color, posterior)[0];
            toSink = emFG->predict2(color, posterior)[0];

            graph.addTermWeights( vtxIdx, fromSource, toSink );

            // set n-weights
            if( p.x>0 ) { // left
                cv::Vec3d diff = col - (cv::Vec3d)eimg.at<cv::Vec3b>(p.y,p.x-1);
                double w = std::exp(-diff.dot(diff));
                graph.addEdges( vtxIdx, vtxIdx-1, w, w );
            }

            if( p.x>0 && p.y>0 ) { // upleft
                cv::Vec3d diff = col - (cv::Vec3d)eimg.at<cv::Vec3b>(p.y-1,p.x-1);
                double w = std::exp(-diff.dot(diff));
                graph.addEdges( vtxIdx, vtxIdx-eimg.cols-1, w, w );
            }

            if( p.y>0 ) { // up
                cv::Vec3d diff = col - (cv::Vec3d)eimg.at<cv::Vec3b>(p.y-1,p.x);
                double w = std::exp(-diff.dot(diff));
                graph.addEdges( vtxIdx, vtxIdx-eimg.cols, w, w );
            }
            if( p.x<eimg.cols-1 && p.y>0 ) { // upright
                cv::Vec3d diff = col - (cv::Vec3d)eimg.at<cv::Vec3b>(p.y+1,p.x+1);
                double w = std::exp(-diff.dot(diff));
                graph.addEdges( vtxIdx, vtxIdx-eimg.cols+1, w, w );
            }
        }
    }
    graph.maxFlow();

    cv::Mat mask(eimg.size(), CV_8UC1, cv::Scalar(0));

    for( p.y = 0; p.y < mask.rows; p.y++ ) {
        for( p.x = 0; p.x < mask.cols; p.x++ ) {
            if( graph.inSourceSegment( p.y*mask.cols+p.x /*vertex index*/ ) )
                mask.at<uchar>(p) = 255;
            else
                mask.at<uchar>(p) = 0;
        }
    }

    cv::imshow("MASK", mask);

    cv::waitKey(0);

    return 0;
}

sel getWin(cv::VideoCapture &cap) {
    sel trsel;
    trsel.state = false;

    cv::Mat currentFrame;
    cap >> currentFrame;

    cv::imshow("CONT", currentFrame);

    trsel.track = false;
    cv::setMouseCallback("CONT", onMouse, (void *)&trsel);

    while ( cap.isOpened() ) {
        cap >> currentFrame;

        trsel.wid = currentFrame.cols;
        trsel.hei = currentFrame.rows;

        if (trsel.track==true) break;
        cv::imshow("CONT", currentFrame);
        trsel.im = currentFrame.clone();

        int key = cv::waitKey(30);
        if (char(key)=='q') break;
    }
    cv::destroyAllWindows();

    return trsel;
}

void onMouse( int event, int x, int y, int, void* s) {
    sel * ss = (sel *)s;
    if( ss->state ) {
        ss->w.x = MIN(x, ss->origin.x);
        ss->w.y = MIN(y, ss->origin.y);
        ss->w.width = std::abs(x - ss->origin.x);
        ss->w.height = std::abs(y - ss->origin.y);
        ss->w &= cv::Rect(0, 0, ss->wid, ss->hei);
    }
    switch( event ) {
        case cv::EVENT_LBUTTONDOWN:
            ss->origin = cv::Point(x,y);
            ss->w = cv::Rect(x,y,0,0);
            ss->state = true;
        break;

        case cv::EVENT_LBUTTONUP:
            ss->state = false;
            ss->track = true;
        break;
    }
}
