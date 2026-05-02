***ONLY PART OF THE CODE***

#include <windows.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <iostream>
#include <vector>
#include <cstdint>
#include <string>
#include <cmath>

// ============================================================
//  PART 1: FPGA UART KONVOLUSYON FONKSIYONLARI
// ============================================================

bool processChannelOnFPGA(HANDLE hSerial, const cv::Mat& channel, cv::Mat& result)
{
    memcpy(result.ptr<uint8_t>(0), channel.ptr<uint8_t>(0), 256);
    memcpy(result.ptr<uint8_t>(255), channel.ptr<uint8_t>(255), 256);

    DWORD written = 0;
    WriteFile(hSerial, channel.ptr<uint8_t>(0), 256, &written, NULL);
    WriteFile(hSerial, channel.ptr<uint8_t>(1), 256, &written, NULL);
    WriteFile(hSerial, channel.ptr<uint8_t>(2), 256, &written, NULL);

    DWORD total_read = 0;
    std::vector<uint8_t> rx_line(256);
    while (total_read < 256) {
        DWORD read = 0;
        ReadFile(hSerial, rx_line.data() + total_read,
            256 - total_read, &read, NULL);
        if (read == 0) break;
        total_read += read;
    }
    if (total_read == 256) memcpy(result.ptr<uint8_t>(1), rx_line.data(), 256);

    for (int row = 3; row < 256; row++) {
        WriteFile(hSerial, channel.ptr<uint8_t>(row), 256, &written, NULL);
        total_read = 0;
        while (total_read < 256) {
            DWORD read = 0;
            ReadFile(hSerial, rx_line.data() + total_read,
                256 - total_read, &read, NULL);
            if (read == 0) break;
            total_read += read;
        }
        if (total_read == 256) {
            memcpy(result.ptr<uint8_t>(row - 1), rx_line.data(), 256);
        }
    }
    return true;
}

cv::Mat fpgaProcess(HANDLE hSerial, const cv::Mat& img, int kernelType)
{
    std::vector<cv::Mat> bgr_channels;
    cv::split(img, bgr_channels);

    std::vector<cv::Mat> output_channels(3);
    const char* names[] = { "B", "G", "R" };

    for (int ch = 0; ch < 3; ch++) {
        std::cout << "  Kanal " << names[ch] << "... " << std::flush;
        cv::Mat result(256, 256, CV_8U);
        processChannelOnFPGA(hSerial, bgr_channels[ch], result);
        output_channels[ch] = result;
        std::cout << "OK\n";
    }

    cv::Mat final_output;
    cv::merge(output_channels, final_output);
    cv::medianBlur(final_output, final_output, 1);

    // Kernel'e gore farkli denoising parametreleri
    int h_val, hColor_val, templateSize, searchSize;
    switch (kernelType) {
    case 0:  // Gaussian
        h_val = 5;  hColor_val = 5;  templateSize = 7;  searchSize = 21;
        break;
    case 1:  // Sharpening (en iyi sonuc 5-10-7-21)
        h_val = 5;  hColor_val = 10; templateSize = 7;  searchSize = 21;
        break;
    case 2:  // Edge Detection
        h_val = 5;  hColor_val = 5;  templateSize = 7;  searchSize = 21;
        break;
    case 3:  // Emboss (en guclu denoising)
        h_val = 25; hColor_val = 25; templateSize = 7;  searchSize = 21;
        break;
    default:
        h_val = 10; hColor_val = 10; templateSize = 7;  searchSize = 21;
        break;
    }

    cv::Mat denoised;
    cv::fastNlMeansDenoisingColored(final_output, denoised,
        h_val, hColor_val, templateSize, searchSize);
    return denoised;
}

// ============================================================
//  PART 2: PC TARAFINDA FFT/DFT TABANLI GORUNTU FILTRELEME
// ============================================================

void swapQuadrants(cv::Mat& mat)
{
    int cx = mat.cols / 2;
    int cy = mat.rows / 2;
    cv::Mat q0(mat, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(mat, cv::Rect(cx, 0, cx, cy));
    cv::Mat q2(mat, cv::Rect(0, cy, cx, cy));
    cv::Mat q3(mat, cv::Rect(cx, cy, cx, cy));
    cv::Mat tmp;
    q0.copyTo(tmp); q3.copyTo(q0); tmp.copyTo(q3);
    q1.copyTo(tmp); q2.copyTo(q1); tmp.copyTo(q2);
}

// FFT filtre tipleri:
// 0 = Low-Pass
// 1 = High-Pass
// 2 = Band-Pass
// 3 = Notch (band-stop)
// 4 = Gaussian Low-Pass (yumusak gecisli)
// 5 = Gaussian High-Pass
// 6 = Emboss-benzeri (HP + bias)
cv::Mat fftFilterChannel(const cv::Mat& channel, int filterType,
    double radius1, double radius2 = 0.0)
{
    cv::Mat floatImg;
    channel.convertTo(floatImg, CV_32F);

    int m = cv::getOptimalDFTSize(channel.rows);
    int n = cv::getOptimalDFTSize(channel.cols);

    cv::Mat padded;
    cv::copyMakeBorder(floatImg, padded,
        0, m - channel.rows, 0, n - channel.cols,
        cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planeArr[2];
    planeArr[0] = padded.clone();
    planeArr[1] = cv::Mat::zeros(padded.size(), CV_32F);
    cv::Mat spectrum;
    cv::merge(planeArr, 2, spectrum);
    cv::dft(spectrum, spectrum);

    int rows = spectrum.rows;
    int cols = spectrum.cols;
    int cx = cols / 2;
    int cy = rows / 2;

    cv::Mat mask = cv::Mat::zeros(rows, cols, CV_32F);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            double dist = std::sqrt((r - cy) * (r - cy) + (c - cx) * (c - cx));
            float val = 0.0f;
            switch (filterType) {
            case 0: val = (dist <= radius1) ? 1.0f : 0.0f; break;  // LP
            case 1: val = (dist > radius1) ? 1.0f : 0.0f; break;  // HP
            case 2: val = (dist >= radius1 && dist <= radius2) ? 1.0f : 0.0f; break;  // BP
            case 3: val = (dist <= radius1 || dist >= radius2) ? 1.0f : 0.0f; break;  // Notch
            case 4: val = std::exp(-(dist * dist) / (2.0 * radius1 * radius1)); break;  // Gaussian LP
            case 5: val = 1.0f - (float)std::exp(-(dist * dist) / (2.0 * radius1 * radius1)); break;  // Gaussian HP
            case 6: // Emboss-benzeri: HP + faz/genlik manipulasyonu
                val = (dist > radius1) ? 1.5f : 0.3f;
                break;
            default: val = 1.0f;
            }
            mask.at<float>(r, c) = val;
        }
    }

    swapQuadrants(mask);

    std::vector<cv::Mat> ch;
    cv::split(spectrum, ch);
    ch[0] = ch[0].mul(mask);
    ch[1] = ch[1].mul(mask);
    cv::Mat filtered;
    cv::merge(ch, filtered);

    cv::Mat result;
    cv::dft(filtered, result, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
    cv::Mat cropped = result(cv::Rect(0, 0, channel.cols, channel.rows)).clone();

    cv::Mat output;
    cv::normalize(cropped, output, 0, 255, cv::NORM_MINMAX);
    output.convertTo(output, CV_8U);
    return output;
}

