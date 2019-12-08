#include <exploration_libraly/struct.h>
#include <exploration_libraly/utility.h>
#include <Eigen/Geometry>
#include <nav_msgs/MapMetaData.h>

namespace ExpLib{
    namespace Struct{
        scanStruct::scanStruct(int size){
            ranges.reserve(size);
            angles.reserve(size);
            x.reserve(size);
            y.reserve(size);
        }

        listStruct::listStruct():duplication(Enum::DuplicationStatus::NOT_DUPLECATION){};
        listStruct::listStruct(const geometry_msgs::Point& p):point(p),duplication(Enum::DuplicationStatus::NOT_DUPLECATION){};

        mapSearchWindow::mapSearchWindow(const geometry_msgs::Point& cc, const nav_msgs::MapMetaData& info, double lx, double ly){ // cc : 検索窓の中心座標, info : 地図のメタデータ, lx,ly : 検索窓の辺の長さ(m)
            if(lx < info.resolution) lx = info.resolution;
            if(ly ==  0.0) ly = lx;
            else if(ly < info.resolution) ly = info.resolution;
            Eigen::Vector2i index(Utility::coordinateToMapIndex(cc,info));
            calcWindowSize(index.x(),index.y(),info.width, info.height, lx/info.resolution, ly/info.resolution);
        }
        mapSearchWindow::mapSearchWindow(const int cx, const int cy, const int mx, const int my, int lx, int ly){ // cx,cy : 検索窓の中心の二次元配列インデックス, mx,my : 地図の辺の長さ(cell), lx,ly : 検索窓の辺の長さ(cell)
            if(lx < 1) lx = 1;
            if(ly == 0) ly = lx;
            else if(ly<  1) ly = 1;
            calcWindowSize(cx,cy,mx,my,lx,ly);
        }

        void mapSearchWindow::calcWindowSize(const int cx, const int cy, const int mx, const int my, const int lx, const int ly){
            int hx1 = lx/2;
            int hx2 = lx%2 == 1 ? lx/2 : lx/2-1; 
            int hy1 = ly/2;
            int hy2 = ly%2 == 1 ? ly/2 : ly/2-1;
            top = cy < hy1 ? 0 : cy-hy1;
            bottom = cy+hy2 > my-1 ? my-1 : cy+hy2;
            left = cx < hx1 ? 0 : cx-hx1; 
            right = cx+hx2 > mx-1 ? mx-1 : cx+hx2;
            width = right-left+1;
            height = bottom-top+1;
        }
    }
}