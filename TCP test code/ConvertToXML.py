#write to xml
import xml.etree.ElementTree as ET

data = '<CARRIERDATA><PLCID>STPLC_10</PLCID><DATEANDTIME>DT#2024-04-23-10:08:58</DATEANDTIME><CARRIERID>7</CARRIERID></CARRIERDATA>\x00H\xbbl@0\xf8%\x00 \xc2%\x00\x00\x00\x00\x00`\xbbl@(\xc1b@`\xbb\x00'
data = data.split('</CARRIERDATA>')[0] + '</CARRIERDATA>'
print(data)
root = ET.fromstring(data)
tree = ET.ElementTree(root)

# Write the XML tree to a file
tree.write("test.xml")
