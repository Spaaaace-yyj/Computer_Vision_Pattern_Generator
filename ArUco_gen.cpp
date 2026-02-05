/**
 * Author: 杨昱杰
 * Date: 2026-02-05
 * Description: 标定板/ARUCO码生成程序
 * Copyright (c) 2026
 */
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv.hpp>
#include <iostream>

constexpr double DPI = 300.0;
constexpr double MM_PER_INCH = 25.4;

int mm2px(double mm) {
    return static_cast<int>(mm / MM_PER_INCH * DPI + 0.5);
}

void drawRuler(cv::Mat& img, cv::Point origin, double length_mm, int height_px) {
    int length_px = mm2px(length_mm);

    cv::line(
        img,
        origin,
        cv::Point(origin.x + length_px, origin.y),
        cv::Scalar(0),
        2
    );

    for (int mm = 0; mm <= static_cast<int>(length_mm); ++mm) {
        int x = origin.x + mm2px(mm);

        int tick_height;
        int thickness;

        if (mm % 10 == 0) {
            tick_height = height_px;
            thickness = 2;
        }
        else if (mm % 5 == 0) {
            tick_height = height_px * 0.6;
            thickness = 2;
        }
        else {
            tick_height = height_px * 0.3;
            thickness = 1;
        }

        cv::line(
            img,
            cv::Point(x, origin.y),
            cv::Point(x, origin.y - tick_height),
            cv::Scalar(0),
            thickness
        );

        if (mm % 10 == 0) {
            cv::putText(
                img,
                std::to_string(mm),
                cv::Point(x - 10, origin.y + 25),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0),
                1,
                cv::LINE_AA
            );
        }
    }

    cv::putText(
        img,
        "mm",
        cv::Point(origin.x + length_px + 10, origin.y + 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0),
        1,
        cv::LINE_AA
    );
}

cv::Mat generateChessboard(
    int rows,
    int cols,
    double square_mm,
    int margin_mm
)
{
    int board_rows = rows + 1;
    int board_cols = cols + 1;

    int square_px = mm2px(square_mm);
    int margin_px = mm2px(margin_mm);

    int width_px = board_cols * square_px + 2 * margin_px;
    int height_px = board_rows * square_px + 2 * margin_px;

    cv::Mat chessboard(height_px, width_px, CV_8UC1, cv::Scalar(255));

    for (int r = 0; r < board_rows; ++r)
    {
        for (int c = 0; c < board_cols; ++c)
        {
            if ((r + c) % 2 == 0)
                continue;

            int x = margin_px + c * square_px;
            int y = margin_px + r * square_px;

            cv::rectangle(
                chessboard,
                cv::Rect(x, y, square_px, square_px),
                cv::Scalar(0),
                cv::FILLED
            );
        }
    }

    //cv::rectangle(
    //    chessboard,
    //    cv::Rect(margin_px, margin_px,
    //        board_cols * square_px,
    //        board_rows * square_px),
    //    cv::Scalar(0),
    //    2
    //);

    return chessboard;
}

typedef enum {
	ARUCO,
	CHESSBOARD_TILE,
    CHESSBOARD
} PatternType;;


int main() {

	PatternType pattern = CHESSBOARD;

    int a4_width_px = mm2px(280);
    int a4_height_px = mm2px(200);

    cv::Mat canvas(a4_height_px, a4_width_px, CV_8UC1, cv::Scalar(255));
    double marker_mm = 40.0;
    double marker_size_mm = 50.0;
    double spacing_mm = 2.0;
    double init_x_mm = 15.0;
    double init_y_mm = 10.0;

    int marker_size_px = mm2px(marker_size_mm);
    int marker_px = mm2px(marker_mm);
    int spacing_px = mm2px(spacing_mm);
    int init_x_px = mm2px(init_x_mm);
    int init_y_px = mm2px(init_y_mm);

    int delta = (int)((marker_size_px - marker_px) / 2);

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    if (pattern == CHESSBOARD)
    {
        int rows = 7;
        int cols = 11;
        double square_mm = 20.0;
        int margin_mm = 0;

        cv::Mat chess = generateChessboard(
            rows,
            cols,
            square_mm,
            margin_mm
        );

        int offset_x = (canvas.cols - chess.cols) / 2;
        int offset_y = (canvas.rows - chess.rows) / 2;

        chess.copyTo(
            canvas(cv::Rect(offset_x, offset_y, chess.cols, chess.rows))
        );

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << square_mm;

        std::string text =
            "Chessboard | " +
            std::to_string(rows) + "x" + std::to_string(cols) +
            " | square size : " + oss.str() + "mm | HBUT L-Create | RoboMaster";

        int font = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 1.5;
        int thickness = 2;
        int baseline = 0;

        cv::Size text_size = cv::getTextSize(
            text, font, scale, thickness, &baseline
        );

        int text_x = (a4_width_px - text_size.width) / 2;
        int text_y = a4_height_px - baseline - mm2px(2);

        cv::putText(
            canvas,
            text,
            cv::Point(text_x, text_y),
            font,
            scale,
            cv::Scalar(0),
            thickness,
            cv::LINE_AA
        );

        drawRuler(
            canvas,
            cv::Point(mm2px(20), mm2px(10)),
            100.0,
            mm2px(3)
        );

        cv::imwrite("Chessboard_A4_print.png", canvas);
        return 0;
    }

    int id = 1;

    for (int y = init_y_px; y + marker_size_px < canvas.rows; y += marker_size_px + spacing_px)
    {
        for (int x = init_x_px; x + marker_size_px < canvas.cols; x += marker_size_px + spacing_px)
        {
            cv::Mat tile_result(marker_size_px, marker_size_px, CV_8UC1, cv::Scalar(255));
            std::string text;

            switch (pattern)
            {
            case ARUCO:
            {
                cv::Mat marker;
                cv::aruco::generateImageMarker(dictionary, id, marker_px, marker, 1);

                marker.copyTo(
                    tile_result(cv::Rect(delta, delta, marker_px, marker_px))
                );

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1) << marker_mm;

                text = "6X6 | " + oss.str() + "mm | ID:" + std::to_string(id) + " | HBUT L-Create";
                id++;
                break;
            }

            case CHESSBOARD_TILE:
            {
				int rows = 8;
				int cols = 11;
                float board_size = 4;

                cv::Mat chess = generateChessboard(
                    rows - 1, cols - 1,
                    board_size,
                    0
                );

                int offset_x = (marker_size_px - chess.cols) / 2;
                int offset_y = (marker_size_px - chess.rows) / 2;

                chess.copyTo(
                    tile_result(cv::Rect(offset_x, offset_y, chess.cols, chess.rows))
                );

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1) << board_size;

                text = "Chessboard | size:" + oss.str() + "mm | " + std::to_string(rows) + "x" + std::to_string(cols) + " | HBUT L-Create";
                break;
            }

            default:
                break;
            }

            int font = cv::FONT_HERSHEY_SIMPLEX;
            double scale = 0.6;
            int thickness = 1;
            int baseline = 0;

            cv::Size text_size = cv::getTextSize(
                text, font, scale, thickness, &baseline
            );

            int text_x = (marker_size_px - text_size.width) / 2;
            int text_y = marker_size_px - baseline - 10;

            cv::putText(
                tile_result,
                text,
                cv::Point(text_x, text_y),
                font,
                scale,
                cv::Scalar(0),
                thickness,
                cv::LINE_AA
            );

            cv::rectangle(
                tile_result,
                cv::Rect(0, 0, marker_size_px, marker_size_px),
                cv::Scalar(0),
                1
            );

            tile_result.copyTo(
                canvas(cv::Rect(x, y, marker_size_px, marker_size_px))
            );

            if (pattern == ARUCO && id >= 250)
                break;
        }

        if (pattern == ARUCO && id >= 250)
            break;
    }


    drawRuler(canvas, cv::Point(mm2px(20), canvas.rows - mm2px(20)), 100.0, mm2px(8));

    cv::imwrite("ArUco_A4_print.png", canvas);

    return 0;
}
