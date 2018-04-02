#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

static void Debayer(const vector<string>& imagelist)
{
    int i,j,k, nimages = (int)imagelist.size();
    int h,w;
    Mat img, tmp_img;
    for(i = 0 ; i < nimages ; ++i) 
    {
        const string& filename = imagelist[i];
        img = imread(filename, 0);
        h = img.size().height;
        w = img.size().width;
        if(img.empty())
            break;
        tmp_img = Mat::zeros(h/2, w/2);
        
    }
    
}


int main( void )
{
    string filename = "../data/camera/export_img/image_list.xml";
    vector<string> imagelist;
    bool ok = readStringList(filename, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << filename << " or the string list is empty" << endl;
    }


}