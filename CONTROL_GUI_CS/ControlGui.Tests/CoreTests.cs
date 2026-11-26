using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using ControlGui.Core.TueUsb;
using ControlGui.Core.Paxini;
using ControlGui.Core.Haptic;
using ControlGui.Core.Motor;
using ControlGui.Core.Recording;
using ControlGui.Core.Analysis;

namespace ControlGui.Tests
{
    [TestClass]
    public class CoreTests
    {
        [TestMethod]
        public void TestCrc8SelfTest()
        {
            Assert.IsTrue(Crc8.SelfTest(), "SAE J1850 CRC8 test vector verification failed");
        }

        [TestMethod]
        public void TestFrameSerializationAndDeserialization()
        {
            byte[] payload = new byte[] { 0x11, 0x22, 0x33, 0x44 };
            var frame = new Frame(FrameType.Req, CanId.StepJogXY, payload, seq: 42);

            byte[] wireBytes = frame.ToBytes();
            Assert.AreEqual(FramingConstants.FrameOverhead + payload.Length, wireBytes.Length);
            Assert.AreEqual(FramingConstants.SOF1, wireBytes[0]);
            Assert.AreEqual(FramingConstants.SOF2, wireBytes[1]);
            Assert.AreEqual(FramingConstants.EOF1, wireBytes[^2]);
            Assert.AreEqual(FramingConstants.EOF2, wireBytes[^1]);

            var parsed = Frame.FromBytes(wireBytes);
            Assert.AreEqual(FrameType.Req, parsed.Type);
            Assert.AreEqual(CanId.StepJogXY, parsed.CanId);
            Assert.AreEqual((byte)42, parsed.Seq);
            Assert.AreEqual(payload.Length, parsed.Dlc);
            CollectionAssert.AreEqual(payload, parsed.Payload);
            Assert.IsTrue(parsed.IsCrcValid());
        }

        [TestMethod]
        public void TestFrameSequenceMutationUpdatesCrc()
        {
            var frame = new Frame(FrameType.Req, CanId.SysPing, new byte[] { 1, 2, 3, 4 }, seq: 0);
            byte[] bytesSeq0 = frame.ToBytes();

            frame.Seq = 17;
            frame.Crc = null;
            byte[] bytesSeq17 = frame.ToBytes();

            Assert.AreNotEqual(bytesSeq0[^3], bytesSeq17[^3], "CRC must be different when sequence number changes");

            var parsed = Frame.FromBytes(bytesSeq17);
            Assert.AreEqual((byte)17, parsed.Seq);
            Assert.IsTrue(parsed.IsCrcValid(), "Mutated frame must produce valid wire CRC");
        }

        [TestMethod]
        public void TestFrameParserStreamingBytes()
        {
            var parser = new FrameParser();
            byte[] payload = new byte[] { 0xAA, 0x01, 0x02 };
            var original = new Frame(FrameType.RespOk, CanId.SysPing, payload, seq: 12);
            byte[] wire = original.ToBytes();

            // Feed leading garbage bytes
            Assert.IsNull(parser.FeedByte(0x00));
            Assert.IsNull(parser.FeedByte(0xFF));
            Assert.IsNull(parser.FeedByte(0xAA)); // Candidate SOF1
            Assert.IsNull(parser.FeedByte(0x00)); // Reset

            Frame? parsed = null;
            for (int i = 0; i < wire.Length; i++)
            {
                parsed = parser.FeedByte(wire[i]);
            }

            Assert.IsNotNull(parsed);
            Assert.AreEqual(CanId.SysPing, parsed!.CanId);
            Assert.AreEqual(FrameType.RespOk, parsed.Type);
            Assert.AreEqual((byte)12, parsed.Seq);
            CollectionAssert.AreEqual(payload, parsed.Payload);
        }

        [TestMethod]
        public void TestHapticPayloadSynthesis()
        {
            var synth = new HapticPayloadSynthesizer();

            // Left sensor experiencing +Y force (should blow Valve A)
            var left = new SensorReading(
                T: 1.0, Epoch: 1000.0, Fx: 0.0, Fy: 2.0, Fz: 5.0,
                FMag: 5.385, FShear: 2.0, Mu: 0.4, CopX: 0.0, CopY: 10.0,
                ShearDir: 90.0, Tilt: 21.8, PeakTaxel: 25, PeakFz: 1.5,
                ContactTaxels: 10, Taxels: new float[52, 3]
            );

            // Right sensor experiencing -X force
            var right = new SensorReading(
                T: 1.0, Epoch: 1000.0, Fx: -1.0, Fy: 2.0, Fz: 5.0,
                FMag: 5.477, FShear: 2.236, Mu: 0.447, CopX: 0.0, CopY: 10.0,
                ShearDir: 116.5, Tilt: 24.1, PeakTaxel: 25, PeakFz: 1.5,
                ContactTaxels: 10, Taxels: new float[52, 3]
            );

            byte[] payload = synth.Synthesize(left, right, nowSec: 1.01);
            Assert.AreEqual(8, payload.Length);

            // Valve A (+Y) should have positive quantized force (2.0 N * 10 = 20)
            Assert.AreEqual((byte)20, payload[0]);
            // Valve B (Mid Fz = 5.0 N * 10 = 50)
            Assert.AreEqual((byte)50, payload[1]);
            // Valve C (-Y) should be 0
            Assert.AreEqual((byte)0, payload[2]);
            // Both contacts active
            byte flags = payload[6];
            Assert.IsTrue((flags & HapticConstants.FlagLeftContact) != 0);
            Assert.IsTrue((flags & HapticConstants.FlagRightContact) != 0);

            var encoder = new TueBinaryEncoder();
            byte[] wireFrame = encoder.Encode(payload);
            Assert.AreEqual(20, wireFrame.Length);

            var frame = Frame.FromBytes(wireFrame);
            Assert.AreEqual(HapticConstants.CanId, frame.CanId);
            Assert.AreEqual(8, frame.Dlc);

            // Real PaxiniSensorWorker readings use Stopwatch T (~12.5s) + Unix UTC Epoch (~1.7e9s),
            // while Stm32HapticStreamer passes Unix UTC seconds as nowSec (null).
            double epochNow = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
            var workerLeft = left with { T = 12.5, Epoch = epochNow - 0.02 };
            var workerRight = right with { T = 12.5, Epoch = epochNow - 0.02 };
            byte[] livePayload = synth.Synthesize(workerLeft, workerRight);
            Assert.AreEqual((byte)20, livePayload[0]);
            Assert.AreEqual((byte)50, livePayload[1]);

            // Stale Epoch (> StaleSeconds) must zero out forces cleanly
            var staleLeft = left with { T = 12.5, Epoch = epochNow - 1.0 };
            var staleRight = right with { T = 12.5, Epoch = epochNow - 1.0 };
            byte[] stalePayload = synth.Synthesize(staleLeft, staleRight, nowSec: epochNow);
            Assert.AreEqual((byte)0, stalePayload[0]);
            Assert.AreEqual((byte)0, stalePayload[1]);
        }

        [TestMethod]
        public void TestHapticSnapshotFromPayload()
        {
            byte[] payload = new byte[8] { 15, 45, 0, 10, 0, 0, 0x03, 7 };
            var snap = HapticSnapshot.FromPayload(payload, t: 1.23, age: 0.01);

            Assert.AreEqual(5, snap.Valves.Length);
            Assert.AreEqual(1.5, snap.Valves[0].Force, 0.01); // 15 * 0.1 N
            Assert.AreEqual(4.5, snap.Valves[1].Force, 0.01); // 45 * 0.1 N
            Assert.AreEqual(0.0, snap.Valves[2].Force, 0.01);
            Assert.AreEqual(1.0, snap.Valves[3].Force, 0.01);
            Assert.AreEqual(0.0, snap.Valves[4].Force, 0.01);
            Assert.IsTrue(snap.FlagNames.Contains("L-contact"));
            Assert.IsTrue(snap.FlagNames.Contains("R-contact"));
            Assert.AreEqual(0.80, snap.Valves[0].Voltage, 0.02);
            Assert.AreEqual(0.0, snap.Valves[2].Voltage, 0.001);
        }

        [TestMethod]
        public void TestValveVoltageSimulation()
        {
            Assert.AreEqual(0.0, HapticConstants.ComputeValveVoltage(0.0));
            Assert.AreEqual(0.0, HapticConstants.ComputeValveVoltage(0.20));
            Assert.AreEqual(2.20, HapticConstants.ComputeValveVoltage(20.0), 0.001);
            Assert.AreEqual(2.20, HapticConstants.ComputeValveVoltage(25.5), 0.001);
            Assert.AreEqual(0, HapticConstants.VoltageToDac(0.0));
            Assert.AreEqual((ushort)((2.20 / 3.30) * 4095), HapticConstants.VoltageToDac(2.20));
        }

        [TestMethod]
        public void TestPaxiniLrcCalculation()
        {
            byte[] data = new byte[] { 0x55, 0xAA, 0x0A, 0x00, 0x01, 0x00, 0x79, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01 };
            byte lrc = PaxiniProtocol.CalcUsbLrc(data);
            int sum = 0;
            foreach (var b in data) sum = (sum + b) & 0xFF;
            byte expected = (byte)(((~sum) + 1) & 0xFF);
            Assert.AreEqual(expected, lrc);
        }

        [TestMethod]
        public void TestSensorMetricsCalculation()
        {
            float[,] tax = new float[52, 3];
            tax[10, 2] = 2.5f; // peak contact taxel
            tax[15, 2] = 1.0f;

            var reading = SensorMetrics.Compute(
                t: 0.5,
                epoch: 123456.0,
                fx: 1.5,
                fy: 2.0,
                fz: 5.0,
                taxels: tax,
                coordinates: S2015EliteGeometry.Coordinates,
                taxelThresholdN: 0.1,
                forceThresholdN: 0.2
            );

            Assert.AreEqual(2.5, reading.FShear, 0.001);
            Assert.AreEqual(Math.Sqrt(2.5 * 2.5 + 5.0 * 5.0), reading.FMag, 0.001);
            Assert.AreEqual(0.5, reading.Mu, 0.001);
            Assert.AreEqual(2, (int)reading.ContactTaxels);
            Assert.AreEqual(11, (int)reading.PeakTaxel); // 1-based index 10 + 1
            Assert.IsFalse(double.IsNaN(reading.CopX));
            Assert.IsFalse(double.IsNaN(reading.CopY));
        }

        [TestMethod]
        public void TestRecordingSchemaCounts()
        {
            var masterCols = RecordingSchema.GetMasterColumns();
            // 7 lead + (13 + 52 * 3) * 2 = 7 + 169 * 2 = 345
            Assert.AreEqual(345, masterCols.Count);

            var perSensorCols = RecordingSchema.GetPerSensorColumns();
            // 2 lead + 13 metrics + 52 * 3 = 171
            Assert.AreEqual(171, perSensorCols.Count);
        }

        [TestMethod]
        public void TestTueUsbClientMockTransactions()
        {
            using var client = new TueUsbClient(mock: true);
            client.Connect();
            Assert.IsTrue(client.IsConnected);

            uint pong = client.Ping(1234);
            Assert.AreEqual((uint)1234, pong);

            var info = client.GetInfo();
            Assert.AreEqual("v1.0", info.VersionString);

            var cfg = client.StepGetConfig();
            Assert.AreEqual(80, cfg.XyStepsPerMm);
            Assert.AreEqual(400, cfg.ZStepsPerMm);

            bool enabled = client.StepEnable(true);
            Assert.IsTrue(enabled);

            client.ServoSetPos(1, 420);
            var fb = client.ServoGetFeedback(1);
            Assert.AreEqual(1, fb.Dynamics.ServoId);
            Assert.AreEqual(420, fb.Dynamics.Position);
        }

        [TestMethod]
        public void TestSpiAdcPackingAndMockClient()
        {
            ushort[] channels = { 0x123, 0x456, 0x789, 0xABC, 0xDEF };
            ulong packed = 0;
            for (int i = 0; i < channels.Length; i++)
            {
                packed |= (ulong)(channels[i] & 0xFFF) << (12 * i);
            }
            packed |= 3UL << 60;
            byte[] payload = new byte[8];
            BinaryPrimitives.WriteUInt64LittleEndian(payload, packed);

            var (decoded, status) = Decoders.SensorAdcAll(payload);
            CollectionAssert.AreEqual(channels, decoded);
            Assert.AreEqual((byte)3, status);
            Assert.IsTrue(new SpiAdcFeedback(decoded, 1, status, 2, 0).AdcValid);
            Assert.IsTrue(new SpiAdcFeedback(decoded, 1, status, 2, 0).AdcError);

            var mvFeedback = new SpiAdcFeedback(new ushort[] { 0, 2048, 4095, 100, 200 }, 1, 1, 2, 0);
            var mv = mvFeedback.Millivolts();
            Assert.AreEqual(2048 * 3300.0 / 4095.0, mv[1], 0.001);
            Assert.AreEqual(3300.0, mv[2], 0.001);

            using var client = new TueUsbClient(mock: true);
            client.Connect();
            var ctrl = client.GetSpiAdcStatus();
            Assert.IsTrue(ctrl.Enabled);
            Assert.AreEqual((byte)1, ctrl.BusId);

            var reading = client.ReadSpiAdc();
            Assert.AreEqual(5, reading.Channels.Length);
            Assert.IsTrue(reading.AdcValid);
            Assert.AreEqual(5, reading.Millivolts().Length);
        }

        [TestMethod]
        public void TestSpiAdcConfigureMock()
        {
            using var client = new TueUsbClient(mock: true);
            client.Connect();

            var cfg = client.ConfigureSpiAdc(false, 1, 2, 20);
            Assert.IsFalse(cfg.Enabled);
            Assert.AreEqual((byte)1, cfg.BusId);
            Assert.AreEqual((byte)2, cfg.CsIdx);
            Assert.AreEqual((byte)20, cfg.PeriodMs);

            var again = client.GetSpiAdcStatus();
            Assert.AreEqual((byte)1, again.BusId);
            Assert.AreEqual((byte)2, again.CsIdx);
        }

        [TestMethod]
        public void TestMotorWorkerMockLifecycle()
        {
            using var worker = new MotorWorker("MOCK_PORT", mock: true, pollHz: 30.0);
            worker.Start();

            // Give worker time to complete handshake
            Thread.Sleep(100);

            var snap = worker.Snapshot();
            Assert.IsTrue(snap.Connected);
            Assert.IsTrue(snap.Streaming);
            Assert.AreEqual(80, snap.StepsPerMmXy);
            Assert.AreEqual(400, snap.StepsPerMmZ);
            Assert.IsTrue(snap.AdcAvailable);
            Assert.IsTrue(snap.AdcUpdated);
            Assert.AreEqual(5, snap.AdcChannels.Length);

            worker.ConfigureSpiAdc(true, 1, 1, 20);
            Thread.Sleep(80);
            snap = worker.Snapshot();
            Assert.AreEqual((byte)1, snap.AdcBusId);
            Assert.AreEqual((byte)1, snap.AdcCsIdx);
            Assert.AreEqual((byte)20, snap.AdcPeriodMs);

            worker.Enable(true);
            Thread.Sleep(50);
            snap = worker.Snapshot();
            Assert.IsTrue(snap.Enabled);

            worker.Jog("x", 25.0);
            snap = worker.Snapshot();
            Assert.AreEqual("x", snap.Jogging);

            worker.JogStop();
            Thread.Sleep(30);
            snap = worker.Snapshot();
            Assert.IsNull(snap.Jogging);
        }

        [TestMethod]
        public void TestUnifiedRecorderFileCreation()
        {
            string testDir = Path.Combine(Path.GetTempPath(), "ControlGuiTests_" + Guid.NewGuid());
            Directory.CreateDirectory(testDir);
            string filePath = Path.Combine(testDir, "test_record.csv");

            try
            {
                Func<List<double>> sampleFn = () =>
                {
                    var list = new List<double>();
                    // 5 lead values (X, Y, Z, pos, volt)
                    list.AddRange(new double[] { 10.0, 20.0, 30.0, 100, 12.0 });
                    // Left block (13 metrics + 156 taxels)
                    for (int i = 0; i < 169; i++) list.Add(i * 0.1);
                    // Right block (13 metrics + 156 taxels)
                    for (int i = 0; i < 169; i++) list.Add(i * 0.2);
                    return list;
                };

                using (var recorder = new UnifiedRecorder(filePath, 0.0, sampleFn, rateHz: 50.0))
                {
                    Thread.Sleep(120); // wait for a few samples
                    long rows = recorder.Stop();
                    Assert.IsTrue(rows > 0, "Recorder should have logged rows");
                }

                Assert.IsTrue(File.Exists(filePath));
                var lines = File.ReadAllLines(filePath);
                Assert.IsTrue(lines.Length > 1, "File must contain header plus rows");
                Assert.AreEqual(345, lines[0].Split(',').Length, "Header must contain 345 columns");
            }
            finally
            {
                try { Directory.Delete(testDir, true); } catch { }
            }
        }

        [TestMethod]
        public void TestSqliteRecorderMultipleSessionsSingleDatabase()
        {
            string testDir = Path.Combine(Path.GetTempPath(), "ControlGuiTests_Sqlite_" + Guid.NewGuid());
            Directory.CreateDirectory(testDir);
            string dbPath = Path.Combine(testDir, "recordings.db");

            try
            {
                Func<List<double>> sampleFn = () =>
                {
                    var list = new List<double>();
                    // 5 lead values (X, Y, Z, pos, volt)
                    list.AddRange(new double[] { 10.0, 20.0, 30.0, 100, 12.0 });
                    // Left block (13 metrics + 156 taxels)
                    for (int i = 0; i < 169; i++) list.Add(i * 0.1);
                    // Right block (13 metrics + 156 taxels)
                    for (int i = 0; i < 169; i++) list.Add(i * 0.2);
                    return list;
                };

                // Session 1: export at timestamp 1
                string table1 = "rec_20261109_120000";
                using (var rec1 = new SqliteRecorder(dbPath, table1, 0.0, sampleFn, rateHz: 50.0, meta: "First Test Session"))
                {
                    Thread.Sleep(120);
                    long rows1 = rec1.Stop();
                    Assert.IsTrue(rows1 > 0, "Session 1 should log rows");
                }

                Assert.IsTrue(File.Exists(dbPath), "Single SQLite database file must exist");

                // Session 2: export at timestamp 2 into the SAME SQLite database
                string table2 = "rec_20261109_120500";
                using (var rec2 = new SqliteRecorder(dbPath, table2, 0.0, sampleFn, rateHz: 50.0, meta: "Second Test Session"))
                {
                    Thread.Sleep(120);
                    long rows2 = rec2.Stop();
                    Assert.IsTrue(rows2 > 0, "Session 2 should log rows");
                }

                // Verify only ONE database file exists in directory (plus WAL/SHM temporary files while active)
                var dbFiles = Directory.GetFiles(testDir, "*.db");
                Assert.AreEqual(1, dbFiles.Length, "Exactly 1 SQLite database file should be created");

                // Verify catalog has 2 sessions
                var sessions = PaxiniDataAnalyzer.GetSessionTables(dbPath);
                Assert.AreEqual(2, sessions.Count, "Database should contain exactly 2 recorded session tables");
                Assert.AreEqual(table2, sessions[0].TableName);
                Assert.AreEqual(table1, sessions[1].TableName);

                // Verify loading data from SQLite
                var (left, right) = PaxiniDataAnalyzer.LoadSessionFromSqlite(dbPath, table1);
                Assert.IsTrue(left.TimeRel.Count > 0, "Left sensor should load rows from SQLite table");
                Assert.IsTrue(right.TimeRel.Count > 0, "Right sensor should load rows from SQLite table");
                Assert.IsFalse(double.IsNaN(left.Fz[0]));
            }
            finally
            {
                // Force SQLite connection cleanup before deleting directory
                Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
                GC.Collect();
                GC.WaitForPendingFinalizers();
                try { Directory.Delete(testDir, true); } catch { }
            }
        }

        [TestMethod]
        public void TestPaxiniDataAnalyzerReport()
        {
            var left = new SensorSessionData
            {
                FilePath = "paxini_20261109_120000_L_COM3.csv",
                Tag = "Left",
                TimeRel = new List<double> { 0.1, 0.2, 0.3 },
                Fz = new List<double> { 2.0, 5.0, 3.0 },
                FMag = new List<double> { 2.2, 5.2, 3.2 },
                Mu = new List<double> { 0.2, 0.3, 0.25 }
            };

            var right = new SensorSessionData
            {
                FilePath = "paxini_20261109_120000_R_COM4.csv",
                Tag = "Right",
                TimeRel = new List<double> { 0.1, 0.2, 0.3 },
                Fz = new List<double> { 2.1, 4.8, 3.1 },
                FMag = new List<double> { 2.3, 5.0, 3.3 },
                Mu = new List<double> { 0.22, 0.28, 0.26 }
            };

            string report = PaxiniDataAnalyzer.GenerateDiagnosticReport(left, right);
            StringAssert.Contains(report, "Peak Normal Force (Fz): 5.00 N");
            StringAssert.Contains(report, "Peak Normal Force (Fz): 4.80 N");
            StringAssert.Contains(report, "EXCELLENT / SYMMETRIC");
        }

        [TestMethod]
        public void TestLogSpacedFrequenciesAndSteppedSchedule()
        {
            double[] asc = BandwidthExperimentRunner.BuildLogSpacedFrequencies(0.1, 10.0, 25, descending: false);
            Assert.AreEqual(25, asc.Length);
            Assert.AreEqual(0.1, asc[0], 1e-4);
            Assert.AreEqual(10.0, asc[^1], 1e-4);
            for (int i = 1; i < asc.Length; i++)
                Assert.IsTrue(asc[i] > asc[i - 1], "Ascending log frequencies must strictly increase");

            double[] desc = BandwidthExperimentRunner.BuildLogSpacedFrequencies(0.1, 10.0, 25, descending: true);
            Assert.AreEqual(10.0, desc[0], 1e-4);
            Assert.AreEqual(0.1, desc[^1], 1e-4);

            var cfg = new BandwidthExperimentConfig
            {
                SweepMode = BandwidthSweepMode.SteppedSine,
                TimingMode = SteppedTimingMode.TotalDurationSec,
                TotalDurationSec = 90.0,
                FrequenciesHz = asc,
                CyclesPerFreq = 10,
                SettleCycles = 2
            };
            var sched = BandwidthExperimentRunner.BuildSteppedSchedule(cfg);
            Assert.AreEqual(25, sched.Count);
            double total = 0;
            foreach (var item in sched) total += item.SettleSec + item.MeasureSec;
            Assert.AreEqual(90.0, total, 0.25);
            Assert.AreEqual(90.0, BandwidthExperimentRunner.EstimateDurationSec(cfg), 0.25);
        }

        [TestMethod]
        public void TestChirpPhaseAndDemodulationBodeRecovery()
        {
            // Simulate a 1st-order low-pass plant with fc = 3.0 Hz excited by a 45 s log-chirp (0.1 -> 10 Hz)
            double f0 = 0.1, f1 = 10.0, dur = 45.0, fs = 100.0;
            double fcTrue = 3.0;
            double wc = 2.0 * Math.PI * fcTrue;
            double u0 = 50.0, ua = 25.0;

            int n = (int)(dur * fs);
            var t = new double[n];
            var u = new double[n];
            var y = new double[n];

            double dt = 1.0 / fs;
            double yState = u0 * 20.0;
            int subSteps = 10;
            double dtSub = dt / subSteps;

            for (int i = 0; i < n; i++)
            {
                double ti = i * dt;
                t[i] = ti;
                double phi = BandwidthExperimentRunner.ChirpPhaseRad(ti, f0, f1, dur);
                u[i] = u0 + ua * Math.Sin(phi);
                y[i] = yState;

                for (int s = 0; s < subSteps; s++)
                {
                    double ts = ti + (s + 0.5) * dtSub;
                    double uSub = u0 + ua * Math.Sin(BandwidthExperimentRunner.ChirpPhaseRad(ts, f0, f1, dur));
                    yState += wc * (uSub * 20.0 - yState) * dtSub;
                }
            }

            double[] bins = BandwidthExperimentRunner.BuildLogSpacedFrequencies(f0, f1, 25, descending: false);
            var pts = new List<BodePoint>();
            for (int b = 0; b < bins.Length; b++)
            {
                var pt = BandwidthExperimentRunner.EvaluateChirpBin(t, u, y, f0, f1, dur, ua, bins[b], b);
                if (pt != null) pts.Add(pt);
            }

            Assert.AreEqual(25, pts.Count);
            double aRef = pts[0].AmpMv;
            for (int i = 0; i < pts.Count; i++)
                pts[i] = pts[i] with { MagDb = 20.0 * Math.Log10(pts[i].AmpMv / aRef) };

            double? fcMeas = BandwidthExperimentRunner.ComputeBandwidthFcHz(pts);
            Assert.IsTrue(fcMeas.HasValue, "Should detect -3 dB cutoff frequency");
            Assert.AreEqual(fcTrue, fcMeas!.Value, 0.25, $"Expected fc near {fcTrue} Hz, got {fcMeas.Value:F2} Hz");
        }

        [TestMethod]
        public void TestBandwidthExperimentMockSteppedAndChirp()
        {
            string testDir = Path.Combine(Path.GetTempPath(), "ControlGuiTests_Bw_" + Guid.NewGuid());
            Directory.CreateDirectory(testDir);

            try
            {
                using var worker = new MotorWorker("MOCK_PORT", mock: true, pollHz: 30.0);
                worker.Start();
                Thread.Sleep(120);
                Assert.IsTrue(worker.Snapshot().Connected);

                // 1. Short Stepped-Sine run in TotalDurationSec mode
                string dbStepped = Path.Combine(testDir, "stepped.db");
                var cfgStepped = new BandwidthExperimentConfig
                {
                    ChannelIndex = 0,
                    SweepMode = BandwidthSweepMode.SteppedSine,
                    TimingMode = SteppedTimingMode.TotalDurationSec,
                    TotalDurationSec = 5.0,
                    FreqMinHz = 2.0,
                    FreqMaxHz = 8.0,
                    FreqPoints = 4,
                    FrequenciesHz = BandwidthExperimentRunner.BuildLogSpacedFrequencies(2.0, 8.0, 4, false),
                    SampleRateHz = 50.0,
                    DbPath = dbStepped
                };
                using (var runner = new BandwidthExperimentRunner(worker, cfgStepped))
                {
                    runner.Start();
                    var deadline = DateTime.UtcNow.AddSeconds(12);
                    while (DateTime.UtcNow < deadline)
                    {
                        var st = runner.GetStatus();
                        if (!st.Running && st.State is "completed" or "error") break;
                        Thread.Sleep(50);
                    }
                    Assert.AreEqual("completed", runner.GetStatus().State);
                    Assert.AreEqual(4, runner.GetBodePoints().Count);

                    // Verify u_pct is phase-continuous across step transitions (no vertical jumps)
                    var samples = runner.GetLiveSamples(2000);
                    Assert.IsTrue(samples.Count > 20);
                    double maxAllowedJump = 25.0 * 2.0 * Math.PI * 8.0 * 0.05 + 1.0; // generous dt jitter bound, far below 25..50% step jump
                    for (int i = 1; i < samples.Count; i++)
                    {
                        if (samples[i].StepIdx != samples[i - 1].StepIdx)
                        {
                            double jump = Math.Abs(samples[i].UPct - samples[i - 1].UPct);
                            Assert.IsTrue(jump <= maxAllowedJump,
                                $"Phase jump at step transition {samples[i - 1].StepIdx}->{samples[i].StepIdx}: {jump:F2}%");
                        }
                    }
                }

                // 2. Short Continuous Log-Chirp run
                string dbChirp = Path.Combine(testDir, "chirp.db");
                var cfgChirp = new BandwidthExperimentConfig
                {
                    ChannelIndex = 0,
                    SweepMode = BandwidthSweepMode.Chirp,
                    TotalDurationSec = 5.0,
                    ChirpSettleSec = 0.5,
                    FreqMinHz = 1.0,
                    FreqMaxHz = 8.0,
                    FreqPoints = 10,
                    SampleRateHz = 50.0,
                    DbPath = dbChirp
                };
                using (var runner = new BandwidthExperimentRunner(worker, cfgChirp))
                {
                    runner.Start();
                    var deadline = DateTime.UtcNow.AddSeconds(12);
                    while (DateTime.UtcNow < deadline)
                    {
                        var st = runner.GetStatus();
                        if (!st.Running && st.State is "completed" or "error") break;
                        Thread.Sleep(50);
                    }
                    Assert.AreEqual("completed", runner.GetStatus().State);
                    Assert.IsTrue(runner.GetBodePoints().Count >= 8, "Chirp should produce Bode summary points");
                    double? mockFc = runner.GetStatus().BandwidthFcHz;
                    string summary = string.Join(", ", runner.GetBodePoints().OrderBy(p => p.FreqHz).Select(p => $"{p.FreqHz:F2}:{p.AmpMv:F1}/{p.MagDb:F2}"));
                    Assert.IsTrue(mockFc.HasValue && mockFc.Value > 3.5 && mockFc.Value < 5.8,
                        $"Expected simulated plant fc near 4.5 Hz, got {mockFc?.ToString("F2") ?? "null"} | pts=[{summary}]");
                }

                Assert.IsTrue(File.Exists(dbStepped));
                Assert.IsTrue(File.Exists(dbChirp));
            }
            finally
            {
                Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
                GC.Collect();
                GC.WaitForPendingFinalizers();
                try { Directory.Delete(testDir, true); } catch { }
            }
        }

        [TestMethod]
        public void TestGripperTransitionMockRun()
        {
            string testDir = Path.Combine(Path.GetTempPath(), "ControlGuiTests_GripExp_" + Guid.NewGuid());
            Directory.CreateDirectory(testDir);
            string dbPath = Path.Combine(testDir, "gripper.db");

            try
            {
                using var worker = new MotorWorker("MOCK_PORT", mock: true, pollHz: 30.0);
                worker.Start();
                Thread.Sleep(250);

                Assert.IsTrue(worker.Snapshot().Connected);
                Assert.IsTrue(worker.Snapshot().ServoOnline, "mock servo should be online");

                SensorReading Make(int n)
                {
                    double fz = Math.Min(4.0, n * 0.05);
                    return new SensorReading(
                        n * 0.02, 0, 0, 0, fz, fz, 0, double.NaN,
                        double.NaN, double.NaN, double.NaN, double.NaN,
                        1, fz, 1, null);
                }

                int lc = 0, rc = 0;
                SensorReading FakeLeft() { lc++; return Make(lc); }
                SensorReading FakeRight() { rc++; return Make(rc); }

                var cfg = new GripperTransitionConfig
                {
                    OpenPos = 55,
                    ClosePos = 420,
                    Speed = 2000,
                    Accel = 20,
                    SettleOpenSec = 0.3,
                    HoldCloseSec = 0.3,
                    ReleaseAtEnd = true,
                    SampleRateHz = 40.0,
                    DbPath = dbPath
                };

                using var runner = new GripperTransitionRunner(
                    worker, cfg, FakeLeft, FakeRight, _ => { });

                runner.Start();
                var deadline = DateTime.UtcNow.AddSeconds(12);
                while (DateTime.UtcNow < deadline)
                {
                    var st = runner.GetStatus();
                    if (!st.Running && (st.State == "completed" || st.State == "error")) break;
                    Thread.Sleep(50);
                }

                var final = runner.GetStatus();
                Assert.AreEqual("completed", final.State, final.Error);
                Assert.IsTrue(final.SampleCount > 10, $"expected samples, got {final.SampleCount}");

                var onsets = runner.GetOnsets();
                Assert.IsNotNull(onsets);
                Assert.IsTrue(onsets!.LeftOnsetSec.HasValue, "left sensor onset should be detected");
                Assert.IsTrue(onsets.RightOnsetSec.HasValue, "right sensor onset should be detected");
                Assert.IsTrue(onsets.ValveOnsetSec.HasValue, "valve onset should follow PaXini haptic stream");
                Assert.IsTrue(onsets.LeftOnsetPos > 0);

                Assert.IsTrue(File.Exists(dbPath));
                using var conn = new Microsoft.Data.Sqlite.SqliteConnection($"Data Source={dbPath}");
                conn.Open();
                using var cmd = conn.CreateCommand();
                cmd.CommandText = "SELECT COUNT(*) FROM samples";
                long rows = Convert.ToInt64(cmd.ExecuteScalar());
                Assert.IsTrue(rows > 10, $"expected sample rows, got {rows}");

                cmd.CommandText = "SELECT COUNT(*) FROM transition_summary";
                Assert.IsTrue(Convert.ToInt64(cmd.ExecuteScalar()) > 0);
            }
            finally
            {
                Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
                GC.Collect();
                GC.WaitForPendingFinalizers();
                try { Directory.Delete(testDir, true); } catch { }
            }
        }

        [TestMethod]
        public void TestGripperMotionZAndXMockRun()
        {
            foreach (var mode in new[] { GripperMotionMode.ZMinusReturn, GripperMotionMode.XPlusReturn })
            {
                string testDir = Path.Combine(Path.GetTempPath(), "ControlGuiTests_GripMotion_" + Guid.NewGuid());
                Directory.CreateDirectory(testDir);
                string dbPath = Path.Combine(testDir, $"motion_{mode}.db");

                try
                {
                    using var worker = new MotorWorker("MOCK_PORT", mock: true, pollHz: 30.0);
                    worker.Start();
                    Thread.Sleep(250);

                    SensorReading Make(int n) => new(
                        n * 0.02, 0, 0.4, 0.1, 1.5, 1.6, 0.4, 0.27,
                        double.NaN, double.NaN, double.NaN, double.NaN,
                        1, 1.5, 1, null);

                    int tick = 0;
                    SensorReading Fake() { tick++; return Make(tick); }

                    var cfg = new GripperTransitionConfig
                    {
                        OpenPos = 55,
                        ClosePos = 250,
                        Speed = 2000,
                        Accel = 20,
                        SettleOpenSec = 0.2,
                        HoldCloseSec = 0.2,
                        ReleaseAtEnd = true,
                        MotionMode = mode,
                        MotionDistanceMm = 8.0,
                        HoldAtTargetSec = 0.2,
                        SampleRateHz = 40.0,
                        DbPath = dbPath
                    };

                    Assert.AreEqual(5.0, cfg.XyVelMmS);
                    Assert.AreEqual(50.0, cfg.XyAccelMmS2);
                    Assert.AreEqual(2.0, cfg.ZVelMmS);
                    Assert.AreEqual(50.0, cfg.ZAccelMmS2);

                    using var runner = new GripperTransitionRunner(worker, cfg, Fake, Fake, _ => { });
                    runner.Start();
                    var deadline = DateTime.UtcNow.AddSeconds(12);
                    while (DateTime.UtcNow < deadline)
                    {
                        var st = runner.GetStatus();
                        if (!st.Running && (st.State == "completed" || st.State == "error")) break;
                        Thread.Sleep(40);
                    }

                    var final = runner.GetStatus();
                    Assert.AreEqual("completed", final.State, final.Error);

                    using var conn = new Microsoft.Data.Sqlite.SqliteConnection($"Data Source={dbPath}");
                    conn.Open();
                    using var cmd = conn.CreateCommand();
                    cmd.CommandText = "SELECT DISTINCT phase FROM samples";
                    var phases = new System.Collections.Generic.HashSet<string>();
                    using (var r = cmd.ExecuteReader())
                    {
                        while (r.Read()) phases.Add(r.GetString(0));
                    }
                    Assert.IsTrue(phases.Contains("Closing"));
                    Assert.IsTrue(phases.Contains("MovingOut"));
                    Assert.IsTrue(phases.Contains("Returning"));
                    Assert.IsTrue(phases.Contains("Releasing"));

                    if (mode == GripperMotionMode.ZMinusReturn)
                    {
                        cmd.CommandText = "SELECT MIN(step_z_mm) FROM samples WHERE phase IN ('MovingOut', 'HoldAtTarget')";
                        Assert.AreEqual(-8.0, Convert.ToDouble(cmd.ExecuteScalar()), 0.2);
                        cmd.CommandText = "SELECT step_z_mm FROM samples WHERE phase = 'Releasing' ORDER BY id DESC LIMIT 1";
                        Assert.AreEqual(0.0, Convert.ToDouble(cmd.ExecuteScalar()), 0.2);
                    }
                    else
                    {
                        cmd.CommandText = "SELECT MAX(step_x_mm) FROM samples WHERE phase IN ('MovingOut', 'HoldAtTarget')";
                        Assert.AreEqual(8.0, Convert.ToDouble(cmd.ExecuteScalar()), 0.2);
                        cmd.CommandText = "SELECT step_x_mm FROM samples WHERE phase = 'Releasing' ORDER BY id DESC LIMIT 1";
                        Assert.AreEqual(0.0, Convert.ToDouble(cmd.ExecuteScalar()), 0.2);
                    }
                }
                finally
                {
                    Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                    try { Directory.Delete(testDir, true); } catch { }
                }
            }
        }

        [TestMethod]
        public void TestSupRingMapperAndLogger()
        {
            // Verify I01 paper convention: -Fx->A, Fz->B, +Fx->C, -Fy->D, +Fy->E
            var enc = new TueBinaryEncoder();

            // 1. Slide Down (-Fx) + Squeeze (Fz)
            var pSlideDown = SupRingMapper.BuildPayload(new SupRingState { Fx = -3.0, Fy = 0.0, Fz = 5.0 }, counter: 7);
            Assert.AreEqual(30, pSlideDown[0], "A should encode -Fx (3.0 N = 30)");
            Assert.AreEqual(50, pSlideDown[1], "B should encode Fz (5.0 N = 50)");
            Assert.AreEqual(0,  pSlideDown[2], "C should be 0 when Fx < 0");
            Assert.AreEqual(0,  pSlideDown[3]);
            Assert.AreEqual(0,  pSlideDown[4]);
            Assert.AreEqual(0,  pSlideDown[5]);
            Assert.AreEqual((byte)(HapticConstants.FlagLeftContact | HapticConstants.FlagRightContact | HapticConstants.FlagSlip), pSlideDown[6]);
            Assert.AreEqual(7,  pSlideDown[7]);

            var wire = enc.Encode(pSlideDown);
            Assert.AreEqual(HapticConstants.FrameLen, wire.Length);
            Assert.AreEqual(0xAA, wire[0]);
            Assert.AreEqual(0xAA, wire[^1]);

            // 2. Slide Up (+Fx), Pull (-Fy), Push (+Fy)
            var pSlideUp = SupRingMapper.BuildPayload(new SupRingState { Fx = 2.5, Fz = 1.0 }, 1);
            Assert.AreEqual(0,  pSlideUp[0]);
            Assert.AreEqual(25, pSlideUp[2]);

            var pPull = SupRingMapper.BuildPayload(new SupRingState { Fy = -4.0, Fz = 10.0 }, 2);
            Assert.AreEqual(40, pPull[3], "D should encode -Fy (Pull)");
            Assert.AreEqual(0,  pPull[4]);
            Assert.AreEqual((byte)(HapticConstants.FlagLeftContact | HapticConstants.FlagRightContact), pPull[6], "4/10=0.4 <= 0.50 -> no slip");

            var pPush = SupRingMapper.BuildPayload(new SupRingState { Fy = 4.2, Fz = 10.0 }, 3);
            Assert.AreEqual(0,  pPush[3]);
            Assert.AreEqual(42, pPush[4], "E should encode +Fy (Push)");

            // 3. Polar mapping check: 0->E, 90->A, 180->D, 270->C, 45->A & E equal
            var (fx0, fy0) = SupRingMapper.FromPolar(5.0, 0.0);
            Assert.AreEqual(0.0, fx0, 1e-6);
            Assert.AreEqual(5.0, fy0, 1e-6);

            var (fx90, fy90) = SupRingMapper.FromPolar(5.0, 90.0);
            Assert.AreEqual(-5.0, fx90, 1e-6);
            Assert.AreEqual(0.0, fy90, 1e-6);

            var (fx180, fy180) = SupRingMapper.FromPolar(5.0, 180.0);
            Assert.AreEqual(0.0, fx180, 1e-6);
            Assert.AreEqual(-5.0, fy180, 1e-6);

            var (fx270, fy270) = SupRingMapper.FromPolar(5.0, 270.0);
            Assert.AreEqual(5.0, fx270, 1e-6);
            Assert.AreEqual(0.0, fy270, 1e-6);

            var (fx45, fy45) = SupRingMapper.FromPolar(10.0, 45.0);
            var p45 = SupRingMapper.BuildPayload(new SupRingState { Fx = fx45, Fy = fy45 }, 0);
            Assert.AreEqual(p45[0], p45[4], 1);

            // 4. Manual flags + ReservedByte
            var pMan = SupRingMapper.BuildPayload(new SupRingState
            {
                AutoFlags = false,
                ManualFlags = HapticConstants.FlagTare | HapticConstants.FlagSlip,
                ReservedByte = 0x5A
            }, 99);
            Assert.AreEqual(0x5A, pMan[5]);
            Assert.AreEqual((byte)(HapticConstants.FlagTare | HapticConstants.FlagSlip), pMan[6]);
            Assert.AreEqual(99, pMan[7]);

            // 5. SQLite Logger
            string testDir = Path.Combine(Path.GetTempPath(), "ControlGuiTests_SupRing_" + Guid.NewGuid());
            Directory.CreateDirectory(testDir);
            string dbPath = Path.Combine(testDir, "sup_ring_test.db");
            try
            {
                using (var logger = new SupRingSqliteLogger())
                {
                    logger.Start(dbPath, 100.0, 3300.0);
                    logger.Enqueue(new SupRingLogSample(0.01, 1.0, -3.0, 0.0, 5.0, 3.0, 90.0, pSlideDown, new ushort[] { 100, 200, 300, 400, 500 }, new double[] { 80.6, 161.2, 241.8, 322.3, 402.9 }));
                    logger.Enqueue(new SupRingLogSample(0.02, 2.0, 0.0, 4.2, 10.0, 4.2, 0.0, pPush, new ushort[] { 101, 201, 301, 401, 501 }, new double[] { 81.4, 162.0, 242.6, 323.1, 403.7 }));
                    Thread.Sleep(120);
                    logger.Stop();
                    Assert.AreEqual(2, logger.RowsLogged);
                }

                using var conn = new Microsoft.Data.Sqlite.SqliteConnection($"Data Source={dbPath}");
                conn.Open();
                using var cmd = conn.CreateCommand();
                cmd.CommandText = "SELECT COUNT(*) FROM samples";
                Assert.AreEqual(2L, Convert.ToInt64(cmd.ExecuteScalar()));

                cmd.CommandText = "SELECT byte_a, byte_b, byte_e FROM samples ORDER BY id ASC LIMIT 1";
                using var r = cmd.ExecuteReader();
                Assert.IsTrue(r.Read());
                Assert.AreEqual(30, r.GetInt32(0));
                Assert.AreEqual(50, r.GetInt32(1));
                Assert.AreEqual(0, r.GetInt32(2));
            }
            finally
            {
                Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
                GC.Collect();
                GC.WaitForPendingFinalizers();
                try { Directory.Delete(testDir, true); } catch { }
            }
        }

        [TestMethod]
        public void TestHapticMappingConfigProtocolAndMock()
        {
            byte[] encoded = Encoders.HapticSetCfg(0.85, 1.95, 0.35, 15.0, saveEeprom: true);
            Assert.AreEqual(8, encoded.Length);
            var decoded = Decoders.HapticMapping(encoded);
            Assert.AreEqual(0.85, decoded.VMinV, 1e-3);
            Assert.AreEqual(1.95, decoded.VMaxV, 1e-3);
            Assert.AreEqual(0.35, decoded.FThreshN, 1e-2);
            Assert.AreEqual(15.00, decoded.FRatedN, 1e-2);
            Assert.IsTrue(decoded.SavedInEeprom);

            Assert.AreEqual(0.0, decoded.ComputeVoltage(0.35), 1e-6);
            double midForce = 0.5 * (0.35 + 15.0);
            Assert.AreEqual(0.5 * (0.85 + 1.95), decoded.ComputeVoltage(midForce), 1e-3);
            Assert.AreEqual(1.95, decoded.ComputeVoltage(25.0), 1e-6);

            string eq = decoded.FormatEquationSummary();
            StringAssert.Contains(eq, "k =");
            StringAssert.Contains(eq, "0.85");
            StringAssert.Contains(eq, "1.95");

            using var client = new TueUsbClient("MOCK_PORT", mock: true);
            client.Connect();
            var initial = client.HapticGetConfig();
            Assert.AreEqual(HapticConstants.VMin, initial.VMinV, 1e-3);
            Assert.AreEqual(HapticConstants.VMax, initial.VMaxV, 1e-3);
            Assert.IsFalse(initial.SavedInEeprom);

            var updated = client.HapticSetConfig(0.90, 2.05, 0.30, 12.0, saveEeprom: true);
            Assert.AreEqual(0.90, updated.VMinV, 1e-3);
            Assert.AreEqual(2.05, updated.VMaxV, 1e-3);
            Assert.AreEqual(0.30, updated.FThreshN, 1e-2);
            Assert.AreEqual(12.00, updated.FRatedN, 1e-2);
            Assert.IsTrue(updated.SavedInEeprom);

            var readBack = client.HapticGetConfig();
            Assert.AreEqual(0.90, readBack.VMinV, 1e-3);
            Assert.IsTrue(readBack.SavedInEeprom);
            client.Disconnect();

            using var worker = new MotorWorker("MOCK_PORT");
            worker.Start();
            try
            {
                DateTime deadline = DateTime.UtcNow.AddSeconds(3);
                while (!worker.Snapshot().Connected && DateTime.UtcNow < deadline)
                    Thread.Sleep(20);
                Assert.IsTrue(worker.Snapshot().Connected);

                uint seqBefore = worker.Snapshot().HapticMappingSeq;
                worker.SetHapticMapping(0.80, 1.80, 0.25, 10.0, false);
                deadline = DateTime.UtcNow.AddSeconds(3);
                while (worker.Snapshot().HapticMappingSeq == seqBefore && DateTime.UtcNow < deadline)
                    Thread.Sleep(20);

                var snap = worker.Snapshot();
                Assert.IsTrue(snap.HapticMappingOk);
                Assert.IsNotNull(snap.HapticMapping);
                Assert.AreEqual(0.80, snap.HapticMapping!.VMinV, 1e-3);
                Assert.AreEqual(1.80, snap.HapticMapping.VMaxV, 1e-3);
                Assert.IsFalse(snap.HapticMapping.SavedInEeprom);

                seqBefore = snap.HapticMappingSeq;
                worker.ReadHapticMapping();
                deadline = DateTime.UtcNow.AddSeconds(3);
                while (worker.Snapshot().HapticMappingSeq == seqBefore && DateTime.UtcNow < deadline)
                    Thread.Sleep(20);
                Assert.AreEqual(10.00, worker.Snapshot().HapticMapping!.FRatedN, 1e-2);
            }
            finally
            {
                worker.Stop();
            }
        }
    }
}
