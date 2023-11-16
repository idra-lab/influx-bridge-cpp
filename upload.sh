# curl --request POST \
#     "http://localhost:8086/api/v2/write?org=IDRA&bucket=MAGICIAN&precision=ns" \
#     --header "Authorization: Token iq7DeSyoJDPKbZ_5vmCb2LjtIPaoL4_qYXYGVw4S8uHA4np_gQEqt5G90DgGSCVBLrdrNL50LZoPd4JU30qRhg==" \
#     --header "Content-Type: text/plain; charset=utf-8" \
#     --header "Accept: application/json" \
#     --data-binary '
#         my_meas,location="Rome" t1=25.0,t2=26.0
#         my_meas,location="Trento" t1=26.0,t2=23.0
#         '
curl --request POST \
    "http://localhost:8086/api/v2/write?org=IDRA&bucket=MAGICIAN&precision=ns" \
    --header "Authorization: Token iq7DeSyoJDPKbZ_5vmCb2LjtIPaoL4_qYXYGVw4S8uHA4np_gQEqt5G90DgGSCVBLrdrNL50LZoPd4JU30qRhg==" \
    --header "Content-Type: text/plain; charset=utf-8" \
    --header "Accept: application/json" \
    --data-binary @points.txt
