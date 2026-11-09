using System;
using System.Collections.Generic;

namespace ControlGui.Core.Paxini
{
    public class RingBuffer
    {
        private readonly int _capacity;
        private readonly int _nCols;
        private readonly double[] _data; // Flattened 2 * capacity * nCols
        private int _index = 0;
        private int _count = 0;
        private readonly object _lock = new object();

        public RingBuffer(int capacity = 65536, int nCols = 14)
        {
            _capacity = capacity;
            _nCols = nCols;
            _data = new double[2 * _capacity * _nCols];
            Array.Fill(_data, double.NaN);
        }

        public void Append(double[] row)
        {
            lock (_lock)
            {
                int offset1 = _index * _nCols;
                int offset2 = (_index + _capacity) * _nCols;
                int copyLen = Math.Min(row.Length, _nCols);

                Array.Copy(row, 0, _data, offset1, copyLen);
                Array.Copy(row, 0, _data, offset2, copyLen);

                _index = (_index + 1) % _capacity;
                if (_count < _capacity)
                {
                    _count++;
                }
            }
        }

        public List<double[]> GetWindow(double tFrom)
        {
            var result = new List<double[]>();
            lock (_lock)
            {
                if (_count == 0) return result;

                // Contiguous slice of length _count ends at (_index + _capacity)
                int end = _index + _capacity;
                int start = end - _count;

                for (int i = start; i < end; i++)
                {
                    double t = _data[i * _nCols];
                    if (t >= tFrom)
                    {
                        double[] row = new double[_nCols];
                        Array.Copy(_data, i * _nCols, row, 0, _nCols);
                        result.Add(row);
                    }
                }
            }
            return result;
        }

        public void Clear()
        {
            lock (_lock)
            {
                _index = 0;
                _count = 0;
                Array.Fill(_data, double.NaN);
            }
        }
    }
}
