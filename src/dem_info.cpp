#include <gdal_priv.h>
#include <cpl_conv.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: dem_info <path.tif>\n";
        return 1;
    }

    GDALAllRegister();

    GDALDataset* ds = (GDALDataset*)GDALOpen(argv[1], GA_ReadOnly);
    if (!ds) {
        std::cerr << "failed to open " << argv[1] << "\n";
        return 1;
    }

    int w = ds->GetRasterXSize();
    int h = ds->GetRasterYSize();
    std::cout << "size: " << w << " x " << h << "\n";

    double gt[6];
    ds->GetGeoTransform(gt);
    std::cout << "origin lon: " << gt[0] << "  lat: " << gt[3] << "\n";
    std::cout << "pixel deg:  " << gt[1] << ", " << gt[5] << "\n";

    GDALRasterBand* band = ds->GetRasterBand(1);
    std::vector<float> elev((size_t)w * h);

    CPLErr err = band->RasterIO(GF_Read, 0, 0, w, h,
                                elev.data(), w, h, GDT_Float32, 0, 0);
    if (err != CE_None) {
        std::cerr << "raster read failed\n";
        GDALClose(ds);
        return 1;
    }

    float lo = elev[0], hi = elev[0];
    double sum = 0.0;
    for (float e : elev) {
        if (e < lo) lo = e;
        if (e > hi) hi = e;
        sum += e;
    }

    std::cout << "elev min: " << lo << " m\n";
    std::cout << "elev max: " << hi << " m\n";
    std::cout << "elev mean: " << sum / elev.size() << " m\n";

    GDALClose(ds);
    return 0;
}
