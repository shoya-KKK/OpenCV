# include <iostream>
# include <string>
# include <sstream>
# include <iomanip>
# include <opencv2/opencv.hpp>
# include <opencv2/imgcodecs.hpp>
# include <vector>

using namespace std;

# define IMAGE_NUM  (10)         /* 画像数 */
# define PAT_ROW    (6)          /* パターンの行数(チェスボードのマス) */
# define PAT_COL    (9)         /* パターンの列数 */
# define PAT_SIZE   (PAT_ROW*PAT_COL)
# define ALL_POINTS (IMAGE_NUM*PAT_SIZE)
# define CHESS_SIZE (32)       /* パターン1マスの1辺サイズ[mm] */

int main(int argc, char* argv[])
{
    int i, j, k;
    int corner_count, found;
    int p_count[IMAGE_NUM];
    // cv::Mat src_img[IMAGE_NUM];
    vector<cv::Mat> srcImages;
    cv::Size pattern_size = cv::Size2i(PAT_COL, PAT_ROW);
    vector<cv::Point2f> corners;
    vector<vector<cv::Point2f>> img_points;
    vector<vector<cv::Point2f>> img_points2;

    // (1)キャリブレーション画像の読み込み
    for (i = 0; i < IMAGE_NUM; i++)
    {
        ostringstream ostr;
        ostr << setfill('0') << setw(2) << i << ".jpg";
        cv::Mat src = cv::imread(ostr.str());
        if (src.empty())
        {
            cerr << "cannot load image file : " << ostr.str() << endl;
        }
        else
        {
            srcImages.push_back(src);
        }
    }

    // (2)3次元空間座標の設定

    vector<cv::Point3f> object;
    for (j = 0; j < PAT_ROW; j++)
    {
        for (k = 0; k < PAT_COL; k++)
        {
            cv::Point3f p(
                j * CHESS_SIZE,
                k * CHESS_SIZE,
                0.0);
            object.push_back(p);
        }
    }

    vector<vector<cv::Point3f>> obj_points;
    vector<vector<cv::Point3f>> obj_points2;
    for (i = 0; i < IMAGE_NUM; i++)
    {
        if (i != IMAGE_NUM - 1)
            obj_points.push_back(object);
        else
            obj_points2.push_back(object);
    }

    // ３次元の点を ALL_POINTS * 3 の行列(32Bit浮動小数点数:１チャンネル)に変換する 


    // (3)チェスボード（キャリブレーションパターン）のコーナー検出
    int found_num = 0;
    cv::namedWindow("Calibration", cv::WINDOW_AUTOSIZE);
    for (i = 0; i < IMAGE_NUM; i++)
    {
        auto found = cv::findChessboardCorners(srcImages[i], pattern_size, corners);
        if (found)
        {
            cout << setfill('0') << setw(2) << i << "... ok" << endl;
            found_num++;
        }
        else
        {
            cerr << setfill('0') << setw(2) << i << "... fail" << endl;
        }

        // (4)コーナー位置をサブピクセル精度に修正，描画
        cv::Mat src_gray = cv::Mat(srcImages[i].size(), CV_8UC1);
        cv::cvtColor(srcImages[i], src_gray, cv::COLOR_BGR2GRAY);
        cv::find4QuadCornerSubpix(src_gray, corners, cv::Size(3, 3));
        cv::drawChessboardCorners(srcImages[i], pattern_size, corners, found);
        if (i != IMAGE_NUM - 1)
            img_points.push_back(corners);
        else
            img_points2.push_back(corners);


        //cv::imshow("Calibration", srcImages[i]);
        //cv::waitKey(0);
    }
    cv::destroyWindow("Calibration");

    if (found_num != IMAGE_NUM)
    {
        cerr << "Calibration Images are insufficient." << endl;
        return -1;
    }


    // (5)内部パラメータ，歪み係数の推定
    cv::Mat cam_mat; // カメラ内部パラメータ行列
    cv::Mat dist_coefs; // 歪み係数
    vector<cv::Mat> rvecs, tvecs; // 各ビューの回転ベクトルと並進ベクトル
    cv::calibrateCamera(
        obj_points,
        img_points,
        srcImages[0].size(),
        cam_mat,
        dist_coefs,
        rvecs,
        tvecs
    );

    std::cout << "cal" << endl;

    cv::Mat rvec2, tvec2;
    cv::solvePnP(//3Dオブジェクトの座標系とカメラの座標系間の変換行列を求る（透視投影行列を求めている）
        obj_points2[0],
        img_points2[0],
        cam_mat,
        dist_coefs,
        rvec2,
        tvec2,
        false,
        cv::SOLVEPNP_ITERATIVE//←この関数は3次元座標と2次元画像が与えられたときにカメラの位置と方向を求める
        //反復的な最適化アルゴリズムを使用して、カメラ位置と方向を求めます。
        //この関数は、カメラが静止している場合や、オブジェクトの位置が既知の場合に有効
    );

    //3D座標系の軸（x, y, z軸）を定義し、その座標をベクトルaxisesに追加
    std::cout << "回転ベクトル"<<rvec2 << endl;//推定された回転ベクトル rvec2 を出力
    vector<cv::Point3f> axises;//cv::Point3fは、3次元空間内の1つの点を表すクラス．x、y、zの3つの要素をもつ
    cv::Point3f origin(0, 0, 0), xaxis(100, 0, 0), yaxis(0, 100, 0), zaxis(0, 0, 100);
    //原点を含む4つを axises という配列に格納
    axises.push_back(origin);
    axises.push_back(xaxis);
    axises.push_back(yaxis);
    axises.push_back(zaxis);
    std::cout << axises << endl;
    std::cout << axises << endl;

    for (i = 0; i < 1; i++) {//forループを用いて、軸を画像上に投影
        vector<cv::Point2f> repro_img_points; //repro_img_pointsという配列を宣言
        cv::projectPoints(//axises（投影軸）の各点を rvec2，tvec2を使って画像上に再投影
            axises,
            rvec2,//回転ベクトル
            tvec2,//平行移動ベクトル
            cam_mat,
            dist_coefs,
            repro_img_points,
            cv::noArray(),
            0
        );
        std::cout << repro_img_points << endl;//再投影した点(軸の点たち）の2D座標は、repro_img_pointsという配列に格納
        for (j = 0; j < 3; j++)
        {
            //画像上に直線を描画するための関数です。主に、2つの点間に直線を描画するために使用
            //入力画像，線の始点，線の終点，色，太さ(デフォ1),直線の種類(デフォ8),座標値の小数点以下の桁数(デフォ0)
            cv::line(srcImages[IMAGE_NUM - 1], cv::Point(repro_img_points[0]), cv::Point(repro_img_points[j + 1]), cv::Scalar(j % 3 * 255, (j + 1) % 3 * 255, (j + 2) % 3 * 255), 5, 8, 0);
        }
        cv::imshow("project", srcImages[IMAGE_NUM - 1]);//表示
        cv::waitKey(0);
    }


    return 0;
}

/*
このプログラムは、先に取得したカメラのキャリブレーションパラメータを使用して、3次元座標を画像上の2次元の座標に投影し、
投影された座標を用いて3次元オブジェクトを2次元画像上に描画することで、キャリブレーション結果を可視化している。
具体的には、solvePnP関数でカメラの位置姿勢を取得し、その位置に対応する3つの軸を定義し、projectPoints関数で3次元座標を2次元画像上に投影し、
cv::line関数を用いて3軸を描画している。
*/