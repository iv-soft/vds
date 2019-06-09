#include "stdafx.h"
#include "private/cert_control_p.h"
#include "cert_control.h"

char vds::cert_control::common_news_channel_id_[65] =
"64KSJ51wjCBYxlWj1OT1tQA5E8xCPPJh5SlSZdT95TE=";
char vds::cert_control::common_news_read_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFTzCCAzegAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NDlaFw0yMDA1MjQwODI4"
"NDlaMDoxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRkwFwYDVQQDDBBDb21tb24gTmV3cyBS"
"ZWFkMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEA3JcH0QDO0aUEqO9UtmSpUwbZSlavo+6m"
"7asaUU1w3CdYZoMWdm/WKriZvSDpVzuVIUCwJ7DOWEhP6FebPHTTMEuIMa304M0+QDsXWfAj5G2eVkWM"
"Cu/wksJqyr8Ev8BglKnKA+I/FBaPMrD1tDBq2RC49+uL0aSjQdMR//9iW52F2EtVs+54ZAE2R9te4Mrb"
"9Q3SbZhixr1QLQZiSm9x6Gd78R0O5Scug5A/Ky+2qm0hXocVL11y2V69OPYQkDace7Yji9fX98WCATe1"
"3qYwYbXxgL429yTLOC62ZjeWwcRt5EOLfIJ6Una1XLISGP23icEXcY+kXu+wFc7WOrtaUZubblVVdCYJ"
"CCtps/RURxKvnr6eQgObQC6hhamuCddPyWZ742gti4zfrpvl4FJS44aPSSUS34TFVQJlv+7ey7RrWuQw"
"2498EIVWWjEzwNAUexFGPZGyKFK+S4EB6pGnuZoV8EGIjxEmsFRCUeFQ0k3s32aTDaSPjbiMrXJmQXiB"
"vr0fBv8aYXekLokHapa/rrgFThnThsNy8N+w8FLlXl+BGrvcEPqj9SbfFEZr+GnkCr2nFJUKSM5I5je+"
"ua08PFbPHLis9wuBipRRPZtUS176jmWIevg4LeXSM+8BSaBePjfExg3zCS+q7f0bKfGCDiWUUQ5LCB5+"
"uieHoph8hq8CAwEAAaNgMF4wDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwEQYJYIZIAYb4"
"QgEBBAQDAgIEMCgGCWCGSAGG+EIBDQQbFhlleGFtcGxlIGNvbW1lbnQgZXh0ZW5zaW9uMA0GCSqGSIb3"
"DQEBCwUAA4ICAQAo6I1A309cqmxdeJq0fZdeKc3/TazYBEGzKRzkyt8j2kMUtUpxXkkDSDP7xKfl+Bh6"
"Yad0yGdkv9AwwvdzwYyVOsnwKLKfjE+g7o+sLUmcMxVrRCG5ZCGzPB1FXZFNXAUAfXfqU77fzIdPr0U9"
"Xm/rb3f7WPT9oxi4aAsNr5MPEgdoEel9LY4uhDmQqF1+xSEwousmL81V+9cihw+xl2PSKHTeTLMR3mXQ"
"tb+xO3trh8CcKqXT7ELrucEIC8SbGgj1W5jvoVmyKVYP4KR0I7431qSAKBXJq9VUk3BnNWbLnAuqxbZF"
"isoILb9ycchooEU0rm2ogf6GjBCYhTHbPJdkqypwZxkEOr5VYFFmPjUR4JrIjYlHFmkSVvlZGnYtZczK"
"X7rA6Yy4Xbln3WDRHSX7FaUvOdfrxRziYnCWvPomoQ3aK0e7K6RuI9s4Tqn47D4K1YF/IqilWDTMSmb+"
"nlmMmtNWJ4+wHygARumghvc1oMb9tx8g1D//NTbvbv/auuxCE/dQHoMRsi4aSRDSblK5ZzsqEXBAoMXt"
"9IvJEuV0yCeIQUE69rC5GL+IKsskmOWvw4YF5xoZeT+meiM8YvPZ5uHohxVQK0ReDDq4Y1AQstCRxcja"
"1n9NyqxiL+sHC07VEQP7fwpAUL7ez1GDps/rZygLdtPMF+BXtxcwtJiBcg==";
char vds::cert_control::common_news_read_private_key_[asymmetric_private_key::base64_size + 1] =
"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"
"S0/BV0lMDMcqjA==";
char vds::cert_control::common_news_write_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFUDCCAzigAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTBaFw0yMDA1MjQwODI4"
"NTBaMDsxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRowGAYDVQQDDBFDb21tb24gTmV3cyBX"
"cml0ZTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAKdSO9gwapIAwO78anNvka4t0v0Nf4pC"
"nWR5eipmMalmT5ggm3l3fp1mu5ZkVID+R2/cj/73AMiEdKkNZd6A09+tCSCnqu7tF2UA11RMqnJMvAeX"
"FImjVnijYspcAbFTTEu4ml3fal4gTFqdDsyUk4C3QnkRbBZvk1ar80R/Cei7kPEc74qmpvYgJeLzyZ6z"
"1LUL9kK8LYLOAaEZpcNR1LmFhn+lSgcIuVn8kQ4wr8oiScwjw2of3HMYhAXBJjTkvUYYs9hneSblAcRR"
"eHQtVpKF/DQCcFdeB+AbryoSZOqKlsbKkvE5e2CqxRGC0RYDvwfX/JSQWHA7R8cCigVWw1S/TOcoihd7"
"a5E0lIkoGoT5B7xPR9x9hyCRClRyYZ5h/4hJ1xuQ2zqvSIC1BCpRVBtf8Y6/fwXaq3tEqO8aZGfwD1yP"
"1BVAkMGmWwMhYgHZNv954pL2O60/oOMT04GlBi2ZfUilmnyRAbQPUyYfWPLkciuB0Mw4fc7obJqb12vP"
"AyqJjzuejb7/Qs3ABCyGjgB8AQVrlDWVLBLAfMV6vrcDJL05QeFEmWCNRg6W/W98WtayqrhezAZv1Ox0"
"oaZg6AVDT5EGuaN1EmsoBjfdkgssIBrAOvkm/FP+pX1cA2z53JRtNNATpOY3erQi3SmlQp01dg9DBz47"
"ffdqyl0tycDHAgMBAAGjYDBeMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgEGMBEGCWCGSAGG"
"+EIBAQQEAwICBDAoBglghkgBhvhCAQ0EGxYZZXhhbXBsZSBjb21tZW50IGV4dGVuc2lvbjANBgkqhkiG"
"9w0BAQsFAAOCAgEAzDItXaNDYHxw+mXMWUo8a7+3ldYICEbIklEwAtjpkrEPSiZBP4Zxh7QQGOJ2tcx9"
"eAYllwhpBK6Xzim9jheJaVhrWayjBaS1bQQmmp0b+afVndbqrdFO4BcuGHThraDhrEj4vFZYrdBhyBlt"
"rsUXRIKtXl7BnZ3YjG3Kb+1O+Kaoko5lWMsAqzCdVHzBcwUBuEMKRqiptJf3ktuHQYsDF6jwVch9Yxno"
"aUWMxEa1A4e7NLZTLz07wfB5BJpkS5ruJhRVuChAxQdE4Tbai/q2vdsKUVVxMISvb/wtgpLqxB7gurML"
"EyjmteYgfQNHrjkUPCmXMWV5Ul4bkI/yYcAfU9O56fM9AUAA2btTrfDYE7/b+Pporj9/VWWKU1bPto+E"
"R4aRFHK54UbeyPTJt4eUUSPo/CtrH8Ocmgkxg1j6B7obLb0FJyJSfQM2YzpTZrHIaGoP1I6HLr/thxem"
"Rp1gyG2HaqaqYICyIcopP/By8JcyrsxdBt9ChXBhUfbIuU/XPCkmJmA9F9WwzKxCaWihUPffCuHzWtNp"
"QfqFDOT0ao5OgrsAybn6HfssJejXq5QjiuDF3KUEr6/ImmlbNYG9ZrV732rptp0O5iwyrXZiXE6SDf7z"
"kMIx5MJy6fWtB6qiE298eL+LhhF69Yx5HOpKg2D3wykooTdZ7Z1K//VwO6w=";
char vds::cert_control::common_news_admin_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFUDCCAzigAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTBaFw0yMDA1MjQwODI4"
"NTBaMDsxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRowGAYDVQQDDBFDb21tb24gTmV3cyBB"
"ZG1pbjCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAMmK+2+yVcbfYgwmAMy+eH4CTmv6iyfp"
"OjDz7aL36Ce2jOVFb9SkZ+3eAPA7RYAakJAxhzfyAQft+SgKkDo7NlD2p1c+N28alNVB0S/95WJl4XYL"
"7aH8RD56UVisT+ZoQuSoJH+xHockc6w9uBGxFkzBtdmr+HMVR5hHRHzMXpvt4bUu1t4Lz4mKAqxeD+tQ"
"vgIOkzjHvFqUGUmgZPyHtlJbT0Z2ATxc+kaKj2ct7N+VnNolRWqntlwaxNcAon9/I+kiKdHvCnNi2pHj"
"IiwabYNn0RoQCrpa8h5iiSnktV+bBW9seOCPMHHgukvMTimMgJWPQeDhFjwFQndD9teVnyHMziV8Ff1M"
"HsLUeA3GwDa66bD0VZkrZx1Q5JeSQUS/kCx8MUtiHJkbvBz6UhCJgTGYsXFRGsjt49XMl+aub5zyCdRB"
"jey7TU/gVv5+h/P9hBKbNByfuFjpqYKbAMxOsL4SYsWG9BBxu6MVv0UR+5sJoN5jQfzB86qdbKT+ZB/z"
"/XtXUf3ERUx5xcqsXSPuzgQb9nXLWgJ/IAtZw62o7TX9rwduFmdn/Gc6tDzAjF7YMOIxrEKD0hFyyi6/"
"Ick9MxhC8KMzzXcOHVlLrztUVG07HX52FotiSgZ0BWySyOQVUSjeY76jmoK3vSji0g4VnwDJu/D1kb9y"
"1VIJsPviPCV5AgMBAAGjYDBeMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgEGMBEGCWCGSAGG"
"+EIBAQQEAwICBDAoBglghkgBhvhCAQ0EGxYZZXhhbXBsZSBjb21tZW50IGV4dGVuc2lvbjANBgkqhkiG"
"9w0BAQsFAAOCAgEAxs73ysdScMboQ1nc581E5Ll7BFW63LOQtMX96+GqCtNQt8abAKYOjnOfU19Iziky"
"+DvcWhHmlBazA34JZnlrMxcADRcF+fpadELiK+9Z5LuUhkjDPyXHZAVv/JNaJznEB0aZllG9MH3SALYZ"
"BDxmDtHNzD0ecyzC1WNx48Rw7vhcL/ctc7PQd6Z5nQA5MeTPIct+Ju/DQfTrLFo5BoUCFN5sphwNQSjN"
"PaSGdO5p3ivJ4IzznNAdFRH0v19ubDuTbK5QuStiWRypQYXJbRqhqy1N/oFSgFEZ18PLlc0PuQPLTfxJ"
"+FV2devrcLMAX4aOiJgs+oQIWp+DpC4aUPGk79LMgshzin7Kt83AfgGijPdZvbYa7rpqbaYHPsXFgi7D"
"UajzXMNOQWieQqVtaThtUkZ+orJzWlKvt5WeKZOBFtHRbE33/Ae4H01U3Tutrk9oLQgnR49o1BV7NozZ"
"gcBUvnZ5NQagEjDTrQtZR4vUNaP2Gc913TVFR6JwfFgFUiJ2v3IYWICuavGXb3J5HOd8qyJPDEBebCTi"
"wPCZcfxmRa3CQf2Wz6iMwztXQ6zxUpxk53xIgmHMFh+uC7ypyu28HgKpx9JqnpLj484J9E0RPD9I89vp"
"5wtXzH/FSrrDmQLkSWO7xXq5njpjpu8oaGsVmEnQ9qTj8E2t4xB2+UCVfd0=";
char vds::cert_control::autoupdate_channel_id_[65] =
"MdoO4zAP08uRhFvCZrK0gNU/otjuOKed+cEbHBurCn8=";
char vds::cert_control::autoupdate_read_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFTjCCAzagAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTVaFw0yMDA1MjQwODI4"
"NTVaMDkxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRgwFgYDVQQDDA9BdXRvVXBkYXRlIFJl"
"YWQwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIKAoICAQDW5mpyut0t5rq0qXJALzsEs6ei0i3cTiK3"
"iLxWB8dgyZQKKqYK9lHXXZQRggPJmq62mUwBXiBY28CMedu37/x2pQs5q/ER5iX/MQH+4Oyc+UTv3KAq"
"jUDN3Hlcskhotgqm22qIM2bTWoAQUjsZc8fyn2w6yUfIw49QYQb33uNTracvrpks6OjYuPNEJ4aOUP7h"
"9muSAEGfglQkeoTd9ilTucUFMJsZc9C2yoqh1NTvNqY9anZ73IwJsx/PeHcXn+LKgtqYt5My9WQAvaZQ"
"KLMbYuUO/gLbFMvosktezmEd4f11LleYAxlx1IbAetJj85znjF9jtXmr7OvR0Xng8RNvf4gqSrFwYlis"
"+NU0XKuXly35fXGE4vmY3uR+bQmGkn1BEyGpRDF5KOKSQC0derFQPn16VmCoViY4KYNNtW1TzECpsL69"
"Rc7tGjrPq4UA3n8EpVyvJL8AWF0EJWWW4nZ1Qqrant+oKNVHrPFOOvgDUHRctnHg/XOpkHge7fWbFbOk"
"rlQd62WXaA40xJgXI5CXlDGcjK8IeOUaKA4fbZyFLD5bxclrsNz7Ua0O9/aqS+xI64M/YziA8csk1DSD"
"Mi65gA/3UZuWnCzpnOZc955fsf1qxH3TFBNBrv6BvF4MgoM6hGKLg2tbOGaKKAyAHegjnBdD/SNLpYCA"
"RErHkK6nRwIDAQABo2AwXjAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBBjARBglghkgBhvhC"
"AQEEBAMCAgQwKAYJYIZIAYb4QgENBBsWGWV4YW1wbGUgY29tbWVudCBleHRlbnNpb24wDQYJKoZIhvcN"
"AQELBQADggIBACd72GzRQ/Q3yCX9YP+sp4dp7G5jalvKpIVS1CGShkDONcT5dyFhHdLrFuQ99daOzCR4"
"SFlkca76ZLg5TJlf3jctezeegrFqwlHFH2/EgOKkGQoK2wM8587LvfxUkWTnM8czVYoKoDVKyy2zAjeY"
"WlAHE6BYPL2a7HT8XVcFuzu9qyL9cNM485lj8qnSMjiKeJoud7erliO4pcBy36Zgijg2tCLC2SQNVvM7"
"hYk7DZLcuefX8gQiW8AThte9E5X2F0nc42GOVrC6woCAyG/c96Hc4jNzd1vbUYC4w952HQp9Agld9UsB"
"rdmrTKAqCuYX514x5Vuic37OqpPF2stoNYXi9/ZOSbL5CLIId1BHUOXfu5R0S2HisFLtKeFGGtK8n0BN"
"fD4/k2u5ECY70q/zSoNqs5RW66S6f5ftJUf1Qh8ZjpdSNfxRUoPvirzq7hvaK9BFER48pFvzgZfquatV"
"NExaLChrfEygNJqcaVqxbBwSxFiSSvnu4yv8TzGv0ruSFTaIZP2AhCsATcWRIr8c4aDuzmuKGQ3QCVA7"
"pQOkNlhChHV74nuBNF2bm+Eif89ijuaRGTEL2TC7N6JEsv9qcMYoHo6mrPJ67auoypUbZlMD08xt932b"
"Ofn/94CWvEkqIa7ot8FbThDTzHx5zoegu5Ypkx6/HE4CeVJzhBymZRbP";
char vds::cert_control::autoupdate_read_private_key_[asymmetric_private_key::base64_size + 1] =
"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"
"Idn0pLClvD2Zrw==";
char vds::cert_control::autoupdate_write_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFTzCCAzegAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTVaFw0yMDA1MjQwODI4"
"NTVaMDoxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRkwFwYDVQQDDBBBdXRvVXBkYXRlIFdy"
"aXRlMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAnrc/UCsIQSgX6srwy8XsvE8PFWwxgkYG"
"qMt5V8hhCRjYNQw/bqyi1OJGaHA3x7FGZoXyxpTuoc4VtMw6kLZAG55EArVBOgIdqHJiEUcMJcyrXMVK"
"G/8851hHB+6Yngmp5mxNcX8FhU5fmMC1jwN+GcF2gbpxLCMts2rdmoTluIiudfu17SrDiBrxGcobB0Xv"
"QMeCeHJ3L44aOb1h+Syp6GyckvO+Yw3vblfC6ZO3X+e/IUhCvFLXwHnyh7843eCwpDuUpLheAmxkb6On"
"eztxQHFmDvwLt4ZjNhuvPjCVj/aMHTky2nuE8gSVFucLD68UQx1B5rWcrDsMu74e9Zvp48Xwj0wR0nuL"
"fJM2BNjYqPGV5zUTBKkQHggqnu4dHMJEzB2JLX64nySfw2DyFaLRGtWp6oRGGDAffB+qYTZaNAQk6fqZ"
"10OfVpEAjSzFC8r3Pd3xpXsCfMnsMqYOjKkEDv9gLMWLQATwq6nWfAX/jaL/m7vcRyB8l0g2N82WBg6F"
"804i96J8p8ntpuD22S/RkcQii8sVVzcLdI8cjkyAmATSxpfGPWuStB+dw93+f+xgiBuUsv3NmvgsvY8r"
"/OFSyv05euSjVVpo+k6qjb3heum4lg0Z1p2d+bUdhcTFaLK2eoZOIz2T9uwO/lraTvhKRUOefEKTo0fH"
"OQ6NOiv6JWsCAwEAAaNgMF4wDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwEQYJYIZIAYb4"
"QgEBBAQDAgIEMCgGCWCGSAGG+EIBDQQbFhlleGFtcGxlIGNvbW1lbnQgZXh0ZW5zaW9uMA0GCSqGSIb3"
"DQEBCwUAA4ICAQDRzteoEEczzhX/3CyTCm10KH9rPhjmKM+HJBdtDMbO6F82bbHowa617EHW8rMkqFjK"
"01brrIXcR2Ns9nW4AsTVW+A1pFS0MrdGlJ2y7Xp5Wi1tSDVEU4cInOUky6d9WiahW0INTvlgUNpiW+mF"
"pt9yzHnQbZpzbLByI7ZmRZFN05lG199NKzojJ7Ag8RyXP83TEzzLU2oMo6IB/6rzEZRPkReb2Jgm+rR4"
"eVXtwQR7sV1L2T78b2I7tcbSc1Va6mD+wFIOnJ704+usSwEY9CrWMd4nzIPLviQO16AHhUN9xul+NarW"
"b8fu5qAQj7hhSv1Yt3K5n4XUakPINZ1F0m3Q2wfI9PSg3VU1khpSB16bzxQKMQ+YN8CSEXa20iRqCTRp"
"0dy+IQxL8CGa2KGuR0KhFLjHq+vbuOWrF+QxQrheCCujL62saKhRMKtylDadwOf9j37DsVGWghg2rnrM"
"FmYGhB1LiIGL84of4Boiy6hNEjRWDOzz8iBrv4MicqtYHFq9LzF2rGHF3SjskAGXmczQMfYMHiR/akUJ"
"+XP3AQnffTCvsKpBcr8QBOBTY3+Y+WfKA98OdIc2W54sZLe97vjKy7TwZ6qN8kc2Ezsl2GeZksvgRQNI"
"1QEduL5dMJH6S0mUgqVp9GOfEgf382Int1LGYXvp17naNx9G6evx85zsOg==";
char vds::cert_control::autoupdate_admin_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFTzCCAzegAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTVaFw0yMDA1MjQwODI4"
"NTVaMDoxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRkwFwYDVQQDDBBBdXRvVXBkYXRlIEFk"
"bWluMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAtNRNvsJ4qA6ArtM/QHQ4RipTQpQ077Sj"
"vT5lBx5LL+rRc2GZiuvVYW15i9ZtYe/+myV0nhJO7yDKELXvdD3k/VfH3YGaAy7DO+y1cv/WL/3Z+X7R"
"kzbZSkMThRvhvNkdI92Lzn1xq4c8BaQsYmkZto8lkZTxl3wc5EzVeWFoO4ciIHAcCt6QZr57mhndZrJy"
"LlCBEFsCZ9m+OYCTtkuQri95uHrtp0o7YPUkXHdu2Fmozvwl7AyBH4/u1N/eVm9UT+CNYqZjEdHwXfKl"
"avBdmXthZd5xhhxxrb+mw0KBHlYwucvhd6xL53K+5yzTDnrlrWzgn1+btjNXP7js3GDwEqKTBKN5ieVf"
"menWxrWlP8asvUqRAvgr4GwIan2iba7qHBzXANt5ysMQVSwYrxlY/mOvzG7WsE5zONcj78YD9Bd4lRH9"
"BoOE5ABkvYCFj7kk2XuC1ThIhu83i2UmP46naPykhNLTdUNwnSVBvNBAj1JteVcTjPM8pdS+0RPeDHHB"
"Fs+/4f8sV6JR+oSti13YV9sOTIzThzAkMmdUktfq4/yuQZPAVFJLtqk+nYY00tMBrEsJlq79V/ebTarH"
"v3aTp6jU035SNAlYJ8hogGQK/1gmFMMeYsFxR+dcnTyj5P6zDtHpaNmXzkTsu1EzeMFPy4LSmtKJFLub"
"TwzqikE0OOECAwEAAaNgMF4wDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwEQYJYIZIAYb4"
"QgEBBAQDAgIEMCgGCWCGSAGG+EIBDQQbFhlleGFtcGxlIGNvbW1lbnQgZXh0ZW5zaW9uMA0GCSqGSIb3"
"DQEBCwUAA4ICAQAsOhObSLecRMQH8QkI9pgGDA8hc4njqf/EjB9LyWdrWEPlu5DKvocmLU0L7yFqrr+Z"
"QDDGbsrObYm56AGcWFdFTtP98lyC+sfJN1Z4Ayy3ypMrFaH3zeYVURuJtILQP0N/O91Dg10mnYDzDZkG"
"iRbJQ+dAZQ5Trvk/QmNpuj9XXwQsKuioGR62bjil8H7gIyhTqMc/CenRWZJRIqY9Ua1pMm13W+m82Oey"
"eAdYYCTMRhDoFupzy4uc5D9ANYroD93MzfMQ48DDOJf7ZdPcCGP4JwtTGelTtKze1kyRHlJDWcydGpbS"
"sWytzkU6qc1l90ounKOWt3Vo5KHYwJgOsLu47/0bAo2autQ8HWB4LZ8FHDuRDUTpw6ckrCMzWsSYR13x"
"L7+Mg8csQOXeLjP80rtqvNFqGbRoyLEPn8PyKuiDEyWxix4bdBK2t/NG6CL1ScKI/0YRWCECRygnERd3"
"n4BtvYhaIdQ+Pc/Y5nNbJk91pMPZD/6PeyYZU/2LoyOgCunsZHByBEIHl6zUQDKH0eG6oQ0cNy/NN9sj"
"ZqJku2qbc1dk+xmjhXpA0l1EDZEtdFSmk6Mczv9HkCXbcymzCPMJfwHqFfCuNDk68E/4pT/vPBIt4Spm"
"eAnEYdDQJqiTgdB4e3lRgzKjl/yaiFshrkespA0EL8d5UngpSPZlOs5+5w==";
char vds::cert_control::web_channel_id_[65] =
"svLv2tXOWW39ASSqTkkfCLBjmYRek6djYg/JDyOloII=";
char vds::cert_control::web_read_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFRzCCAy+gAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTdaFw0yMDA1MjQwODI4"
"NTdaMDIxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MREwDwYDVQQDDAhXZWIgUmVhZDCCAiIw"
"DQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAN6wECwyYdGBBFnoz6VZHBT/cVygm1qzxqqukkvCOPJF"
"cH9OZYY2MrUnqtx2bOuo9B1AbRTusP9P6No9SNLXSx/ZeM7LvZ1yM9QyQCWj6z0DBeBiJoLcy0MEzScd"
"fpz/z5p0mIvJeG7fS8pnni+wIodtM5LjJMmOvzeZKYkOmvBNKINMSt6+828z7oFpXCtFVJ9KnRP8DWXT"
"wjN/2O5qIcAW31yzaC142+szxSgRbLcx9I8HUN7rRbGZespzhLvA1I8fIn6Ivf2R8l6+gUHt4ObLvyU1"
"0CZtosm+YE30KNqFamk0AMqLBDptrp1Mh/IKAHAVTDUS1SJnG0bKwL+FdQqFRDLTaq8bG3VEf/nAPT3P"
"Q4Uv1R4UkCs+uD4FNp6cSSas143r56aA9bDOMg03Tt2QSQJVxwn87r9H++Ot7Y89DY5Qj5MxQptDFH66"
"Io3Ydl3HhokIrhqfUGkYwar8aS0g38YXHTTsUvODGfwpRkKk9J3EkCQ9ubUnvP29i8+w/1HPouTr6H5e"
"r5eeZmDcPOQ3H8vsHnUCqhGvP2BwumF7xILH2h8O/uU+EFs7c9dxpTj9lwQUr218cWJG7F8hOOu8gXr+"
"glQLE07gMQ+85gd3IJAvqap0u9V3lc1QWw+2yx0ksyD30nhG9DSOB1fbsAXwn/olpQTe6jDQDyrsoXbR"
"AgMBAAGjYDBeMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgEGMBEGCWCGSAGG+EIBAQQEAwIC"
"BDAoBglghkgBhvhCAQ0EGxYZZXhhbXBsZSBjb21tZW50IGV4dGVuc2lvbjANBgkqhkiG9w0BAQsFAAOC"
"AgEAPF1tRYT4Je2M+Upib52LSKnWhJbx+iV6xRJ6zt46yUlH48wtojUMLGoL6Tn1apFFfYQ//GOI1Amd"
"KY5jTloEqzmrRK+GCqlqdqmBvHLStfJiVeFVDC+kdaAZHwAyZF36XuVHwE0Rif9LGma7zPz2tGwuXNbP"
"tlQ4JNZvzGsw8LMDd518oqstKLDWgdQCuO+Xj185KVd3TVem7s1lGZl1jKCPAUPcTnZmHiY+I6F4xXzZ"
"JG6AcGoVuFXpqNkC3W9WaGiMMNrV2wlArpvwIHJ4li4JWPFOa8aF7Q9MWLnAAwTkn+YLbkfXVmnkjNqq"
"FJCGN7W2/4RMrbd15XCc8ihTyjHZ3IuLL5fKv793w+T+2CITaVG16Cf7tpLvlaOye3Jrh136BP4526ZP"
"Rn38qTU/Hr3pDCGmCCI9r9eXQqSgCMT2eRQ9NCp/xeNWlGtsVys5nEZxi93ItdViGH2NPEaLgZ14otgU"
"M1K7us1HySSuKzJO1/6ah15nZwSfT4u4Z+3G8XCdz8hYd4wupJPveU0drAW4Fut0q2b8aWUCZhc24HTW"
"NPCJzkPuUCCcVuJA5dWiaWa5fULQk/oxg2emkYIDdUJFFFSDKVfK72mArplQSK6jWtl0sW//m4jWFkzJ"
"Z7gkvT/etKxLGzJfqES2bmKB9TSVdIx/4pN9xuyWdc1sRx0=";
char vds::cert_control::web_read_private_key_[asymmetric_private_key::base64_size + 1] =
"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"
"y+CfjAZA66I=";
char vds::cert_control::web_write_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFSDCCAzCgAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTdaFw0yMDA1MjQwODI4"
"NTdaMDMxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRIwEAYDVQQDDAlXZWIgV3JpdGUwggIi"
"MA0GCSqGSIb3DQEBAQUAA4ICDwAwggIKAoICAQDb/FqrxXCiQdQ5ywQk0NcXMvA+MbjlmHjt/i7UdDeZ"
"dFJt8us3EkcwnxOq41j9QtMpgvvVyuPDuvOTcFPaAYGbGDz8vB4j8dtJ6xR1KBcoIJOZ5n0t8c+Ldasu"
"UiP3Ts+UQQie1j8UwGOWwxV/yvqF85wIjvBMCvEGSccjhZmRtpFLw4rQO8pMSpw6TrFaD0us81Jc7607"
"zhWUKSUJbMFTVcYD8ZGydXX2DqHXwJG3rwVawbFylyWdxM6jcpeIsS5tyugQy+b9GBuq/wi3m8nyxJTi"
"DXLPZn4kM9hNi2YLzFsInjTp16JSVEfJURsFtAOTJeHl+hsfJ1ZjvlWTaHdTLgs10AsSWALsBIVDm9tS"
"eBJSWVI5eJ4KSLJEkDaR2fOSLEdHjfQK6gyM0LkLGOM+z6938U34mYS5QWVeEuFdneU0mrGzHGpBjcCy"
"bbGFnJMDVolqwZMgbNgHpiC2HDmWPjyW6Q7+ccv3i3KlvVgnDbcLtQ9XwFyPuNVM6p6sU5k1QxvMk8pS"
"30NOYY/uf7KST58pdD89jJQojDhcaBvrj1HfYEYm+JiQvWATS9vTLHckwYSfBIFmSvHfL0IXCon0UhDM"
"Dnxz+eMRucNxJ/HnT9uk02+u/SY8IeJnixd+/3AMayF2W1GanTmVFZoIkww1TQYlgN8ZGYLmZ7+VIqFN"
"swIDAQABo2AwXjAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBBjARBglghkgBhvhCAQEEBAMC"
"AgQwKAYJYIZIAYb4QgENBBsWGWV4YW1wbGUgY29tbWVudCBleHRlbnNpb24wDQYJKoZIhvcNAQELBQAD"
"ggIBAKyIQ0hVFf/OfQCAvA5hlUr6Hw049wdX12M1Ota8vZh+nz6gqYBM8pN1DvN+2Jo+dlKWnSGwJ/Ib"
"kJ8ChAS5jQ9iBkoQXXV2NEmmHOtzn5JEtVawG/z60fUkAQqeqxeAfRiKDFWRpBUfImYQhEjR+UYX9AgG"
"mmX6Iw1XWLnFdtm4KUk9xNc+QZ3S2xOaZJ6BVtqzCwhWVvfCq0amfHJsayJl7LnDW+g7okePmZkWKJ4t"
"Lw7tRff2AGv+UGxAME8iZ4CSxfi6klniTLWW9DeH6MxO98kcWYEGWuvrPw4K6laqc9zcpdOe4QS9NzOM"
"0PDYEbuD7wu77GsWc4tQSJTd0fsdXlKAwr2aa4LtcWiQJYhxJXwUFGNEySiX1X1HYcGmUWTzFBrkRsbn"
"R/TfEGDkqrOGhkb9qN/MK5M8KrIlEJ97UOSujTkkjs78vxj/v6TvgBJoE9DJqNKUyNTzkIxvTUtxNZj3"
"6POd5qnxHXW0EpP/+0xsPgCEXgQHVnTR4XLO1WMSpAQgGkvFb3br/ZJwYTZZAzZr15ZNNQ6BFTRBik1a"
"SAB0KH0rfsU9IMt0ygk3AQiR4+3ZB+024EQXm4TQupghvgcgZomToTPa7fjCl/IiZ41w8QLDHHN4O4N6"
"9dc6N0zH8s6whKpMGsR6pe16GlXzuY/8hexTTitNkrbxIysd";
char vds::cert_control::web_admin_certificate_[asymmetric_public_key::base64_size + 1] =
"MIIFSDCCAzCgAwIBAgIBATANBgkqhkiG9w0BAQsFADA6MQswCQYDVQQGEwJSVTEQMA4GA1UECgwHSVZ5"
"U29mdDEZMBcGA1UEAwwQdmFkaW1AaXYtc29mdC5ydTAeFw0xOTA1MjUwODI4NTdaFw0yMDA1MjQwODI4"
"NTdaMDMxCzAJBgNVBAYTAlJVMRAwDgYDVQQKDAdJVnlTb2Z0MRIwEAYDVQQDDAlXZWIgQWRtaW4wggIi"
"MA0GCSqGSIb3DQEBAQUAA4ICDwAwggIKAoICAQDSEtUm4ox/4fTWIWczFt7mEx48aK6K53A1NvBbOmSY"
"nitIDuf4kvLshpM9sGLlAgYYyNXesxpYV7emz0ZypYmfZ2TFK8UE0GEQVFi0xo5JCzyJ+te3Bn0Y4WIk"
"NnjHXze3TGo/UIWZndU7E77Jb0gXdEz0yi0D50sx5Dqc9W+lUl1oCYfGwqjL8pwvJsadjhJaRAR+8oFl"
"U7VG5y8bqVHfK7bYlNDhbDeI9wUrwXEgMOlFsyR6RQx1UHUBzXTYgMaJs4yUu5Cn3xmq3ogCdTJKsEPV"
"xw66lqCUgyfeueFEqJ3pqZ17qnr3M6S25w7Z7hGZZBBj3+TPnJ6BidZJNU/KA4puUrplycWzTXhfhj64"
"GMEm3J3lYy7c6zI3uzwBjc1BB12bhXY+cuA2RjnaoCU3kl/0/HoLns2FMIzqjSUwRZ430zrOC453HfoK"
"Sc++0jB75WPLdXPyV6RlE2dS0vQ5fXnvoKPufzIBMLL+jidWOwoM1sgJr5yfTfK0dyguAgSwbss+sIyF"
"kziZdaW88CFoSaz7CCIeocL6qltprdECZtt7swOSm8GR99LNtyIQiLccWyYS+j8atZcGIDseGcK5+jht"
"YkHXFV2pCWEB6WSbYEdTyN7X/g8+82+ydg6QoL/dqSbCE22OJDx5J76y8u1/uLzwPKMa4BDX6EbVEb2P"
"rwIDAQABo2AwXjAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBBjARBglghkgBhvhCAQEEBAMC"
"AgQwKAYJYIZIAYb4QgENBBsWGWV4YW1wbGUgY29tbWVudCBleHRlbnNpb24wDQYJKoZIhvcNAQELBQAD"
"ggIBAAuQBF68shltR8hUs9Ph4CZx6gM5eNhk5R+YXGfCAwwdJE3EPV81URXF407fHButpfbWXU2WFQF9"
"u5yL96hPEfqo9JKrSIlj4AFCNyyJQpH+gCN5e5zMxL044DWqATnHXYBmP/IFPS1vjWIATWQiCXM/lRah"
"Zk3lkAUG5csuyT+G5Y6Bdl0FQllhDbHe5DVGryhGEas36FiOBYhhCMb3Ucvn76kw/m9IeM/a9OD/snwb"
"i4t2eNilyQhwWUVFm8J0U6BcTzbhMwfF4pu66X4ohJZiUXixqGZT4xB5YauRQKuQvkDeYqDXZH6Rvb4K"
"YXPcCLbVxkohM74RIi8+2Yn5DWIZZfoMamj/flS/kYzDr/r0fKSTb2P1+VL/2SWsBCAxgRfIcpyifMDY"
"hGsJvhdi87Eduw1rW2J4pY6It+46+UJBGqjAH3yUo0YtwhsR6Vn3UOfb8C1/Sh8NHqCAjPVaQudXUSOY"
"k4BSjPUSuenN0krApIbUVun0IrukjelRt0nl2B0FdC0/idsBKBFnCVdNLSwcbF3vNnlWnD3dh4W2rscp"
"hlcIldboxyHPXZKDK09qfJri4KWlQRksM7UZcamC/LAkx9Hf5YxOMV/pEnhJva/g78iQg4U37BnyeqOU"
"gr5xQBeCzbwBhmfeUk10Q2805NLg0ROwakJdk0rWtYOOFf3P";

/*
 * User: user_id -> certificate (object_id, user_id, parent_id)
 *
 *
 */

//static vds::crypto_service::certificate_extension_type id_extension_type()
//{
//  static vds::crypto_service::certificate_extension_type result = vds::crypto_service::register_certificate_extension_type(
//      "1.2.3.4",
//      "VDS Identifier",
//      "VDS Identifier");
//
//  return result;
//}
//
//static vds::crypto_service::certificate_extension_type parent_id_extension_type()
//{
//  static vds::crypto_service::certificate_extension_type result = vds::crypto_service::register_certificate_extension_type(
//      "1.2.3.5",
//      "VDS Parent Identifier",
//      "VDS Parent Identifier");
//
//  return result;
//}
//
//static vds::crypto_service::certificate_extension_type user_id_extension_type()
//{
//  static vds::crypto_service::certificate_extension_type result = vds::crypto_service::register_certificate_extension_type(
//      "1.2.3.6",
//      "VDS User Identifier",
//      "VDS User Identifier");
//
//  return result;
//}
//
//static vds::crypto_service::certificate_extension_type parent_user_id_extension_type()
//{
//	static vds::crypto_service::certificate_extension_type result = vds::crypto_service::register_certificate_extension_type(
//		"1.2.3.7",
//		"Parent VDS User Identifier",
//		"Parent VDS User Identifier");
//
//	return result;
//}
//
//static vds::guid certificate_parent_id(const vds::certificate & cert)
//{
//  return vds::guid::parse(cert.get_extension(cert.extension_by_NID(parent_id_extension_type())).value);
//}

vds::expected<vds::certificate> vds::_cert_control::create_cert(
  const vds::asymmetric_private_key & private_key) {
  
  GET_EXPECTED(cert_pkey, asymmetric_public_key::create(private_key));
  GET_EXPECTED(name, cert_pkey.hash(hash::sha1()));

  certificate::create_options local_user_options;
  local_user_options.country = "RU";
  local_user_options.organization = "IVySoft";
  local_user_options.name = base64::from_bytes(name);

  return certificate::create_new(cert_pkey, private_key, local_user_options);
}

vds::expected<vds::certificate> vds::_cert_control::create_cert(
	const vds::asymmetric_private_key & private_key,
	const certificate & user_cert,
	const asymmetric_private_key & user_private_key) {
  GET_EXPECTED(cert_pkey, asymmetric_public_key::create(private_key));
  GET_EXPECTED(name, cert_pkey.hash(hash::sha1()));
  
  certificate::create_options local_user_options;
  local_user_options.country = "RU";
  local_user_options.organization = "IVySoft";
  local_user_options.name = base64::from_bytes(name);
  local_user_options.ca_certificate = &user_cert;
  local_user_options.ca_certificate_private_key = &user_private_key;

  return certificate::create_new(cert_pkey, private_key, local_user_options);
}

const std::string& vds::cert_control::auto_update_login() {
  static std::string auto_update_login_("auto_update_login");
  return auto_update_login_;
}

const std::string& vds::cert_control::auto_update_password() {
  static std::string auto_update_password_("auto_update_password");
  return auto_update_password_;
}

const std::string& vds::cert_control::web_login() {
  static std::string web_login_("web_login");
  return web_login_;
}

const std::string& vds::cert_control::web_password() {
  static std::string web_password_("web_password");
  return web_password_;
}

vds::expected<void> vds::cert_control::private_info_t::genereate_all() {
  GET_EXPECTED(key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  this->common_news_write_private_key_ = std::make_shared<asymmetric_private_key>(std::move(key));

  GET_EXPECTED_VALUE(key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  this->common_news_admin_private_key_ = std::make_shared<asymmetric_private_key>(std::move(key));

  GET_EXPECTED_VALUE(key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  this->autoupdate_write_private_key_ = std::make_shared<asymmetric_private_key>(std::move(key));

  GET_EXPECTED_VALUE(key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  this->autoupdate_admin_private_key_ = std::make_shared<asymmetric_private_key>(std::move(key));

  GET_EXPECTED_VALUE(key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  this->web_write_private_key_ = std::make_shared<asymmetric_private_key>(std::move(key));

  GET_EXPECTED_VALUE(key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  this->web_admin_private_key_ = std::make_shared<asymmetric_private_key>(std::move(key));

  return expected<void>();
}

static void save_buffer(char(&buffer_storage)[65], const vds::const_data_buffer & data) {
  auto storage_str = vds::base64::from_bytes(data);
  vds_assert(sizeof(buffer_storage) > storage_str.length());
  strcpy(buffer_storage, storage_str.c_str());
}

static void save_certificate(char (&cert_storage)[vds::asymmetric_public_key::base64_size + 1], const vds::asymmetric_public_key & cert) {
  auto der = cert.der();
  if(der.has_error()) {
    throw std::runtime_error(der.error()->what());
  }

  auto cert_storage_str = vds::base64::from_bytes(der.value());
  vds_assert(sizeof(cert_storage) > cert_storage_str.length());
  strcpy(cert_storage, cert_storage_str.c_str());
}

static void save_private_key(char (&private_key_storage)[vds::asymmetric_private_key::base64_size + 1], const vds::asymmetric_private_key & private_key) {
  auto der = private_key.der(std::string());
  if (der.has_error()) {
    throw std::runtime_error(der.error()->what());
  }

  const auto private_key_str = vds::base64::from_bytes(der.value());
  vds_assert(sizeof(private_key_storage) > private_key_str.length());
  strcpy(private_key_storage, private_key_str.c_str());
}

vds::expected<void> vds::cert_control::genereate_all(
  const private_info_t & private_info) {

  //
  GET_EXPECTED(common_news_read_private_key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  save_private_key(common_news_read_private_key_, common_news_read_private_key);
  GET_EXPECTED(common_news_read_certificate, asymmetric_public_key::create(common_news_read_private_key));
  save_certificate(common_news_read_certificate_, common_news_read_certificate);

  GET_EXPECTED(common_news_write_certificate, asymmetric_public_key::create(*private_info.common_news_write_private_key_));
  save_certificate(common_news_write_certificate_, common_news_write_certificate);

  GET_EXPECTED(common_news_admin_certificate, asymmetric_public_key::create(*private_info.common_news_admin_private_key_));
  save_certificate(common_news_admin_certificate_, common_news_admin_certificate);
  GET_EXPECTED(common_news_channel_id, common_news_admin_certificate.hash(hash::sha256()));
  save_buffer(common_news_channel_id_, common_news_channel_id);

  //Auto update
  GET_EXPECTED(autoupdate_read_private_key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  save_private_key(autoupdate_read_private_key_, autoupdate_read_private_key);
  GET_EXPECTED(autoupdate_read_certificate, asymmetric_public_key::create(autoupdate_read_private_key));
  save_certificate(autoupdate_read_certificate_, autoupdate_read_certificate);

  GET_EXPECTED(autoupdate_write_certificate, asymmetric_public_key::create(*private_info.autoupdate_write_private_key_));
  save_certificate(autoupdate_write_certificate_, autoupdate_write_certificate);

  GET_EXPECTED(autoupdate_admin_certificate, asymmetric_public_key::create(*private_info.autoupdate_admin_private_key_));
  save_certificate(autoupdate_admin_certificate_, autoupdate_admin_certificate);

  GET_EXPECTED(autoupdate_channel_id, autoupdate_admin_certificate.hash(hash::sha256()));
  save_buffer(autoupdate_channel_id_, autoupdate_channel_id);

  //Web
  GET_EXPECTED(web_read_private_key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
  save_private_key(web_read_private_key_, web_read_private_key);
  GET_EXPECTED(web_read_certificate, asymmetric_public_key::create(
    web_read_private_key));
  save_certificate(web_read_certificate_, web_read_certificate);

  GET_EXPECTED(web_write_certificate, asymmetric_public_key::create(
    *private_info.web_write_private_key_));
  save_certificate(web_write_certificate_, web_write_certificate);

  GET_EXPECTED(web_admin_certificate, asymmetric_public_key::create(
    *private_info.web_admin_private_key_));
  save_certificate(web_admin_certificate_, web_admin_certificate);

  GET_EXPECTED(web_channel_id, web_admin_certificate.hash(hash::sha256()));
  save_buffer(web_channel_id_, web_channel_id);

  return expected<void>();
}
