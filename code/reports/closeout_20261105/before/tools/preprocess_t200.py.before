"""Read the original XLSX using only Python's ZIP/XML libraries; never download at runtime."""
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'bluerov2/data/T200-Public-Performance-Data-10-20V-September-2019.xlsx'
NS = {'s': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}


def isotonic(values):
    """Equal-weight PAVA: least-squares nondecreasing magnitudes, outward from neutral."""
    blocks = []
    for value in values:
        blocks.append([float(value), 1])
        while len(blocks) > 1 and blocks[-2][0] > blocks[-1][0]:
            b, a = blocks.pop(), blocks.pop()
            blocks.append([(a[0]*a[1]+b[0]*b[1])/(a[1]+b[1]), a[1]+b[1]])
    return [v for v, n in blocks for _ in range(n)]


def main():
    with ZipFile(SOURCE) as archive:
        strings = [''.join(x.itertext()) for x in ET.fromstring(archive.read('xl/sharedStrings.xml'))]
        def rows(path):
            result = []
            for row in ET.fromstring(archive.read(path)).findall('s:sheetData/s:row', NS):
                cells = []
                for cell in row:
                    value = cell.find('s:v', NS)
                    cells.append(None if value is None else strings[int(value.text)] if cell.get('t') == 's' else float(value.text))
                result.append(cells)
            return result
        notes = '\n'.join(''.join(p.itertext()) for p in ET.fromstring(archive.read('xl/drawings/drawing1.xml')).iter('{http://schemas.openxmlformats.org/drawingml/2006/main}p'))
        sheets = ET.fromstring(archive.read('xl/workbook.xml')).find('s:sheets', NS)
        tables = []
        for number, sheet in enumerate(sheets):
            name = sheet.get('name')
            if name == 'READ ME FIRST':
                continue
            data = rows(f'xl/worksheets/sheet{number+1}.xml')
            headers = [str(x).strip() for x in data[0]]
            assert headers == ['PWM (µs)', 'RPM', 'Current (A)', 'Voltage (V)', 'Power (W)', 'Force (Kg f)', 'Efficiency (g/W)'], headers
            voltage = float(name.split()[0])
            samples = data[1:]
            assert len(samples) == 201 and all(r[0] == 1100+4*i and r[3] == voltage for i, r in enumerate(samples))
            raw = [r[5]*9.80665 for r in samples]
            # Preserve the exact zero region. Correct only sign-inconsistent noise on its own branch.
            positive = isotonic([max(0., f) for f in raw[100:]])
            reverse = isotonic([max(0., -f) for f in raw[100::-1]])
            force = [-f for f in reverse[::-1]][:-1] + positive
            tables.append({'voltage_v': voltage, 'pwm_us': [r[0] for r in samples], 'force_n': force,
                           'max_isotonic_adjustment_n': max(abs(a-b) for a,b in zip(raw, force)),
                           'neutral_zero_pwm_us': [r[0] for r, f in zip(samples, force) if f == 0]})
        output = {'format': 't200-static-v1', 'source_url': 'https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx',
                  'sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(), 'sheets': [s.get('name') for s in sheets],
                  'columns': headers, 'source_notes': notes, 'kgf_to_n': 9.80665, 'neutral_us': 1500,
                  'processing': 'Equal-weight isotonic regression (PAVA) of separate outward branch magnitudes; zero region preserved. Voltage interpolation of force at equal PWM, then branch inversion. Duplicate force: smallest absolute PWM offset; zero anchor at outward edge of deadband. No extrapolation.', 'tables': tables}
        path = ROOT/'bluerov2/data/t200_curves.json'
        path.write_text(json.dumps(output, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps({k:v for k,v in output.items() if k!='tables'}, ensure_ascii=False, indent=2))
        print([(t['voltage_v'], t['force_n'][0],t['force_n'][-1],t['max_isotonic_adjustment_n']) for t in tables])

if __name__ == '__main__':
    main()
