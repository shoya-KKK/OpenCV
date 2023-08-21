# include <iostream>
# include <string>
# include <sstream>
# include <iomanip>
# include <opencv2/opencv.hpp>
# include <opencv2/imgcodecs.hpp>
# include <vector>

using namespace std;

# define IMAGE_NUM  (10)         /* �摜�� */
# define PAT_ROW    (6)          /* �p�^�[���̍s��(�`�F�X�{�[�h�̃}�X) */
# define PAT_COL    (9)         /* �p�^�[���̗� */
# define PAT_SIZE   (PAT_ROW*PAT_COL)
# define ALL_POINTS (IMAGE_NUM*PAT_SIZE)
# define CHESS_SIZE (32)       /* �p�^�[��1�}�X��1�ӃT�C�Y[mm] */

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

    // (1)�L�����u���[�V�����摜�̓ǂݍ���
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

    // (2)3������ԍ��W�̐ݒ�

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

    // �R�����̓_�� ALL_POINTS * 3 �̍s��(32Bit���������_��:�P�`�����l��)�ɕϊ����� 


    // (3)�`�F�X�{�[�h�i�L�����u���[�V�����p�^�[���j�̃R�[�i�[���o
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

        // (4)�R�[�i�[�ʒu���T�u�s�N�Z�����x�ɏC���C�`��
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


    // (5)�����p�����[�^�C�c�݌W���̐���
    cv::Mat cam_mat; // �J���������p�����[�^�s��
    cv::Mat dist_coefs; // �c�݌W��
    vector<cv::Mat> rvecs, tvecs; // �e�r���[�̉�]�x�N�g���ƕ��i�x�N�g��
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
    cv::solvePnP(//3D�I�u�W�F�N�g�̍��W�n�ƃJ�����̍��W�n�Ԃ̕ϊ��s�������i�������e�s������߂Ă���j
        obj_points2[0],
        img_points2[0],
        cam_mat,
        dist_coefs,
        rvec2,
        tvec2,
        false,
        cv::SOLVEPNP_ITERATIVE//�����̊֐���3�������W��2�����摜���^����ꂽ�Ƃ��ɃJ�����̈ʒu�ƕ��������߂�
        //�����I�ȍœK���A���S���Y�����g�p���āA�J�����ʒu�ƕ��������߂܂��B
        //���̊֐��́A�J�������Î~���Ă���ꍇ��A�I�u�W�F�N�g�̈ʒu�����m�̏ꍇ�ɗL��
    );

    //3D���W�n�̎��ix, y, z���j���`���A���̍��W���x�N�g��axises�ɒǉ�
    std::cout << "��]�x�N�g��"<<rvec2 << endl;//���肳�ꂽ��]�x�N�g�� rvec2 ���o��
    vector<cv::Point3f> axises;//cv::Point3f�́A3������ԓ���1�̓_��\���N���X�Dx�Ay�Az��3�̗v�f������
    cv::Point3f origin(0, 0, 0), xaxis(100, 0, 0), yaxis(0, 100, 0), zaxis(0, 0, 100);
    //���_���܂�4�� axises �Ƃ����z��Ɋi�[
    axises.push_back(origin);
    axises.push_back(xaxis);
    axises.push_back(yaxis);
    axises.push_back(zaxis);
    std::cout << axises << endl;
    std::cout << axises << endl;

    for (i = 0; i < 1; i++) {//for���[�v��p���āA�����摜��ɓ��e
        vector<cv::Point2f> repro_img_points; //repro_img_points�Ƃ����z���錾
        cv::projectPoints(//axises�i���e���j�̊e�_�� rvec2�Ctvec2���g���ĉ摜��ɍē��e
            axises,
            rvec2,//��]�x�N�g��
            tvec2,//���s�ړ��x�N�g��
            cam_mat,
            dist_coefs,
            repro_img_points,
            cv::noArray(),
            0
        );
        std::cout << repro_img_points << endl;//�ē��e�����_(���̓_�����j��2D���W�́Arepro_img_points�Ƃ����z��Ɋi�[
        for (j = 0; j < 3; j++)
        {
            //�摜��ɒ�����`�悷�邽�߂̊֐��ł��B��ɁA2�̓_�Ԃɒ�����`�悷�邽�߂Ɏg�p
            //���͉摜�C���̎n�_�C���̏I�_�C�F�C����(�f�t�H1),�����̎��(�f�t�H8),���W�l�̏����_�ȉ��̌���(�f�t�H0)
            cv::line(srcImages[IMAGE_NUM - 1], cv::Point(repro_img_points[0]), cv::Point(repro_img_points[j + 1]), cv::Scalar(j % 3 * 255, (j + 1) % 3 * 255, (j + 2) % 3 * 255), 5, 8, 0);
        }
        cv::imshow("project", srcImages[IMAGE_NUM - 1]);//�\��
        cv::waitKey(0);
    }


    return 0;
}

/*
���̃v���O�����́A��Ɏ擾�����J�����̃L�����u���[�V�����p�����[�^���g�p���āA3�������W���摜���2�����̍��W�ɓ��e���A
���e���ꂽ���W��p����3�����I�u�W�F�N�g��2�����摜��ɕ`�悷�邱�ƂŁA�L�����u���[�V�������ʂ��������Ă���B
��̓I�ɂ́AsolvePnP�֐��ŃJ�����̈ʒu�p�����擾���A���̈ʒu�ɑΉ�����3�̎����`���AprojectPoints�֐���3�������W��2�����摜��ɓ��e���A
cv::line�֐���p����3����`�悵�Ă���B
*/