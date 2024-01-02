#!/usr/bin/env python2
#-*-coding:utf-8-*-

import parser

# sentence = "$GHFPD,1234,12234.123,90.123,45.55,33.2,23.66,127.1111,78.23,1.1,2.3,6.6,1.5,16,23,2,4*68"
# sentence = "$GPGGA,023042,3907.3837,N,12102.4684,W,1,04,2.3,507.3,M,-24.1,M,,*75"

sentence = "$GINS,1451,368123.310,34.1966,108.85,80.3,12.3,34.3,1.4,0.32,-22,90.32,11,1,2,3,4,5,1.1,7,*58"

res = parser.parse_nmea_sentence(sentence)
print(res)