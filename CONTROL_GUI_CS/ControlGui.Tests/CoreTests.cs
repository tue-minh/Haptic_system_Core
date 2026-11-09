using System;
using System.Collections.Generic;
using System.IO;
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
    }
}
