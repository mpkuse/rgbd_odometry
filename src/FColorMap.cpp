#include <FColorMap.h>


FColorMap::FColorMap()
{
    FColorMap(64);
}

FColorMap::FColorMap(int nColors)
{
    assert( nColors > 0 && "expecting +ve false in #Colors");
    colors.reserve(nColors);

    colors[0   ] = cv::Vec3b( 143     ,0     ,0 );
    colors[1   ] = cv::Vec3b( 159     ,0     ,0 );
    colors[2   ] = cv::Vec3b( 175     ,0     ,0 );
    colors[3   ] = cv::Vec3b( 191     ,0     ,0 );
    colors[4   ] = cv::Vec3b( 207     ,0     ,0 );
    colors[5   ] = cv::Vec3b( 223     ,0     ,0 );
    colors[6   ] = cv::Vec3b( 239     ,0     ,0 );
    colors[7   ] = cv::Vec3b( 255     ,0     ,0 );
    colors[8   ] = cv::Vec3b( 255    ,15     ,0 );
    colors[9   ] = cv::Vec3b( 255    ,31     ,0 );
    colors[10   ] = cv::Vec3b( 255    ,47     ,0 );
    colors[11   ] = cv::Vec3b( 255    ,63     ,0 );
    colors[12   ] = cv::Vec3b( 255    ,79     ,0 );
    colors[13   ] = cv::Vec3b( 255    ,95     ,0 );
    colors[14   ] = cv::Vec3b( 255   ,111     ,0 );
    colors[15   ] = cv::Vec3b( 255   ,127     ,0 );
    colors[16   ] = cv::Vec3b( 255   ,143     ,0 );
    colors[17   ] = cv::Vec3b( 255   ,159     ,0 );
    colors[18   ] = cv::Vec3b( 255   ,175     ,0 );
    colors[19   ] = cv::Vec3b( 255   ,191     ,0 );
    colors[20   ] = cv::Vec3b( 255   ,207     ,0 );
    colors[21   ] = cv::Vec3b( 255   ,223     ,0 );
    colors[22   ] = cv::Vec3b( 255   ,239     ,0 );
    colors[23   ] = cv::Vec3b( 255   ,255     ,0 );
    colors[24   ] = cv::Vec3b( 239   ,255    ,15 );
    colors[25   ] = cv::Vec3b( 223   ,255    ,31 );
    colors[26   ] = cv::Vec3b( 207   ,255    ,47 );
    colors[27   ] = cv::Vec3b( 191   ,255    ,63 );
    colors[28   ] = cv::Vec3b( 175   ,255    ,79 );
    colors[29   ] = cv::Vec3b( 159   ,255    ,95 );
    colors[30   ] = cv::Vec3b( 143   ,255   ,111 );
    colors[31   ] = cv::Vec3b( 127   ,255   ,127 );
    colors[32   ] = cv::Vec3b( 111   ,255   ,143 );
    colors[33    ] = cv::Vec3b( 95   ,255   ,159 );
    colors[34    ] = cv::Vec3b( 79   ,255   ,175 );
    colors[35    ] = cv::Vec3b( 63   ,255   ,191 );
    colors[36    ] = cv::Vec3b( 47   ,255   ,207 );
    colors[37    ] = cv::Vec3b( 31   ,255   ,223 );
    colors[38    ] = cv::Vec3b( 15   ,255   ,239 );
    colors[39     ] = cv::Vec3b( 0   ,255   ,255 );
    colors[40     ] = cv::Vec3b( 0   ,239   ,255 );
    colors[41     ] = cv::Vec3b( 0   ,223   ,255 );
    colors[42     ] = cv::Vec3b( 0   ,207   ,255 );
    colors[43     ] = cv::Vec3b( 0   ,191   ,255 );
    colors[44     ] = cv::Vec3b( 0   ,175   ,255 );
    colors[45     ] = cv::Vec3b( 0   ,159   ,255 );
    colors[46     ] = cv::Vec3b( 0   ,143   ,255 );
    colors[47     ] = cv::Vec3b( 0   ,127   ,255 );
    colors[48     ] = cv::Vec3b( 0   ,111   ,255 );
    colors[49     ] = cv::Vec3b( 0    ,95   ,255 );
    colors[50     ] = cv::Vec3b( 0    ,79   ,255 );
    colors[51     ] = cv::Vec3b( 0    ,63   ,255 );
    colors[52     ] = cv::Vec3b( 0    ,47   ,255 );
    colors[53     ] = cv::Vec3b( 0    ,31   ,255 );
    colors[54     ] = cv::Vec3b( 0    ,15   ,255 );
    colors[55     ] = cv::Vec3b( 0     ,0   ,255 );
    colors[56     ] = cv::Vec3b( 0     ,0   ,239 );
    colors[57     ] = cv::Vec3b( 0     ,0   ,223 );
    colors[58     ] = cv::Vec3b( 0     ,0   ,207 );
    colors[59     ] = cv::Vec3b( 0     ,0   ,191 );
    colors[60     ] = cv::Vec3b( 0     ,0   ,175 );
    colors[61     ] = cv::Vec3b( 0     ,0   ,159 );
    colors[62     ] = cv::Vec3b( 0     ,0   ,143 );
    colors[63     ] = cv::Vec3b( 0     ,0   ,127 );
}

cv::Vec3b &FColorMap::at(int i)
{
    assert( i>=0  && i<colors.size() && i++ );
    return colors[i];
}
