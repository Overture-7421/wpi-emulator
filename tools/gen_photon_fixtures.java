// Packs fixture inputs with the REAL PhotonLib serde. Run by tools/gen_photon_fixtures.py (which documents the
// whole fixture workflow); not part of any build.
//
//   java -cp <photontargeting-java, photonlib-java, wpimath, wpiutil, ... jars> tools/gen_photon_fixtures.java < cases
//
// stdin: blocks "fixture <name>" ... "end"; the lines in between are the fixture input (see the Python header).
// stdout (tab separated, nothing else): "version", "maxlist", one "schema" per message (name, UUID, text) and one
// "bytes" per fixture. Problems go to stderr with a non-zero exit status.
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.PhotonVersion;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.common.dataflow.structures.PacketSerde;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

class GenPhotonFixtures {
  static final PacketSerde<PhotonPipelineResult> SERDE = PhotonPipelineResult.photonStruct;

  public static void main(String[] args) throws Exception {
    System.out.println("version\t" + PhotonVersion.versionString);
    System.out.println("maxlist\t" + Packet.MAX_ARRAY_LEN);
    printSchemas();

    BufferedReader in = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));
    String name = null;
    List<String> body = new ArrayList<>();
    String line;
    while ((line = in.readLine()) != null) {
      line = line.strip();
      if (line.startsWith("fixture ")) {
        name = line.substring("fixture ".length()).strip();
        body.clear();
      } else if (line.equals("end")) {
        try {
          System.out.println("bytes\t" + name + "\t" + HexFormat.of().formatHex(pack(body)));
        } catch (RuntimeException e) {
          throw new IllegalStateException("fixture " + name + ": " + e.getMessage(), e);
        }
        name = null;
      } else if (name != null) {
        body.add(line);
      }
    }
  }

  /** Every photon message below the result, with the UUID the library reports and the md5 of its text. */
  static void printSchemas() throws Exception {
    Map<String, PacketSerde<?>> all = new LinkedHashMap<>();
    collect(SERDE, all);
    for (PacketSerde<?> serde : all.values()) {
      String[] type = serde.getTypeString().split(":"); // photonstruct:<Name>:<uuid>
      String schema = serde.getSchema();
      String md5 =
          HexFormat.of()
              .formatHex(MessageDigest.getInstance("MD5").digest(schema.getBytes(StandardCharsets.UTF_8)));
      if (!md5.equals(serde.getInterfaceUUID()) || !type[2].equals(serde.getInterfaceUUID())) {
        throw new IllegalStateException(type[1] + ": md5(schema)=" + md5 + " but the library says " + serde.getInterfaceUUID());
      }
      if (schema.contains("\t") || schema.contains("\n") || schema.contains("\r")) {
        throw new IllegalStateException(type[1] + ": the schema text holds a control character");
      }
      System.out.println("schema\t" + type[1] + "\t" + serde.getInterfaceUUID() + "\t" + schema);
    }
  }

  static void collect(PacketSerde<?> serde, Map<String, PacketSerde<?>> out) {
    if (out.putIfAbsent(serde.getTypeString(), serde) != null) {
      return;
    }
    for (PacketSerde<?> inner : serde.getNestedPhotonMessages()) {
      collect(inner, out);
    }
  }

  /** Cursor over the whitespace separated words of one input line. */
  static final class Words {
    final String[] words;
    int next;

    Words(String line) {
      words = line.split("\\s+");
    }

    String word() {
      if (next >= words.length) {
        throw new IllegalArgumentException("the line ends early: " + String.join(" ", words));
      }
      return words[next++];
    }

    void expect(String keyword) {
      String got = word();
      if (!got.equals(keyword)) {
        throw new IllegalArgumentException("expected '" + keyword + "' but found '" + got + "'");
      }
    }

    double real() {
      return Double.parseDouble(word());
    }

    long integer() {
      return Long.parseLong(word());
    }

    void end() {
      if (next != words.length) {
        throw new IllegalArgumentException("unexpected '" + words[next] + "' at the end of the line");
      }
    }

    /** x y z qw qx qy qz. PhotonLib normalises every Rotation3d it is given, so the quaternion must be a fixed point. */
    Transform3d transform() {
      double x = real();
      double y = real();
      double z = real();
      double w = real();
      double qx = real();
      double qy = real();
      double qz = real();
      Transform3d transform = new Transform3d(new Translation3d(x, y, z), new Rotation3d(new Quaternion(w, qx, qy, qz)));
      Quaternion q = transform.getRotation().getQuaternion();
      if (!same(q.getW(), w) || !same(q.getX(), qx) || !same(q.getY(), qy) || !same(q.getZ(), qz)) {
        throw new IllegalArgumentException(
            "the quaternion is not exactly unit length: PhotonLib packs ("
                + q.getW() + ", " + q.getX() + ", " + q.getY() + ", " + q.getZ() + ") instead of ("
                + w + ", " + qx + ", " + qy + ", " + qz + "); use unit_quaternion() in the generator");
      }
      return transform;
    }

    List<TargetCorner> corners() {
      int count = (int) integer();
      List<TargetCorner> corners = new ArrayList<>();
      for (int i = 0; i < count; i++) {
        double x = real();
        double y = real();
        corners.add(new TargetCorner(x, y));
      }
      return corners;
    }
  }

  static boolean same(double a, double b) {
    return Double.doubleToRawLongBits(a) == Double.doubleToRawLongBits(b);
  }

  static byte[] pack(List<String> body) {
    String message = null;
    long[] meta = null;
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    Optional<MultiTargetPNPResult> multi = Optional.empty();
    Transform3d value = null;
    for (String line : body) {
      if (line.isEmpty() || line.startsWith("#")) {
        continue;
      }
      Words w = new Words(line);
      String kind = w.word();
      switch (kind) {
        case "message" -> {
          message = w.word();
          w.end();
        }
        case "meta" -> {
          meta = new long[] {w.integer(), w.integer(), w.integer(), w.integer()};
          w.end();
        }
        case "target" -> {
          double yaw = w.real();
          double pitch = w.real();
          double area = w.real();
          double skew = w.real();
          int fiducialId = (int) w.integer();
          int objDetectId = (int) w.integer();
          float objDetectConf = Float.parseFloat(w.word());
          double ambiguity = w.real();
          w.expect("best");
          Transform3d best = w.transform();
          w.expect("alt");
          Transform3d alt = w.transform();
          w.expect("rect");
          List<TargetCorner> rect = w.corners();
          w.expect("det");
          List<TargetCorner> det = w.corners();
          w.end();
          PhotonTrackedTarget target =
              new PhotonTrackedTarget(
                  yaw, pitch, area, skew, fiducialId, objDetectId, objDetectConf, best, alt, ambiguity, rect, det);
          // The constructor cuts detectedCorners to 8 items (PhotonTrackedTarget, v2026.3.4), but the wire format allows
          // 127: assign the lists to the public fields so that the serde sees exactly the lists of the fixture.
          target.minAreaRectCorners = rect;
          target.detectedCorners = det;
          targets.add(target);
        }
        case "multi" -> {
          double bestReprojErr = w.real();
          double altReprojErr = w.real();
          double ambiguity = w.real();
          w.expect("best");
          Transform3d best = w.transform();
          w.expect("alt");
          Transform3d alt = w.transform();
          w.expect("ids");
          int count = (int) w.integer();
          List<Short> ids = new ArrayList<>();
          for (int i = 0; i < count; i++) {
            ids.add(Short.parseShort(w.word()));
          }
          w.end();
          // PnpResult(best, alt, ambiguity, bestReprojErr, altReprojErr): not the order of the wire schema
          PnpResult pnp = new PnpResult(best, alt, ambiguity, bestReprojErr, altReprojErr);
          if (!same(pnp.bestReprojErr, bestReprojErr) || !same(pnp.altReprojErr, altReprojErr) || !same(pnp.ambiguity, ambiguity)) {
            throw new IllegalArgumentException("the PnpResult constructor takes its arguments in another order now");
          }
          multi = Optional.of(new MultiTargetPNPResult(pnp, ids));
        }
        case "value" -> {
          value = w.transform();
          w.end();
        }
        default -> throw new IllegalArgumentException("unknown input line: " + line);
      }
    }
    if (message == null) {
      throw new IllegalArgumentException("no 'message' line");
    }
    switch (message) {
      case "PhotonPipelineResult" -> {
        if (meta == null) {
          throw new IllegalArgumentException("no 'meta' line");
        }
        PhotonPipelineResult result = new PhotonPipelineResult(meta[0], meta[1], meta[2], meta[3], targets, multi);
        Packet packet = new Packet(1 << 20);
        SERDE.pack(packet, result);
        byte[] bytes = packet.getWrittenDataCopy();
        PhotonPipelineResult back = SERDE.unpack(new Packet(bytes));
        if (!back.equals(result)) {
          throw new IllegalStateException("the real unpacker does not return what the real packer was given");
        }
        return bytes;
      }
      case "Transform3d" -> {
        if (value == null) {
          throw new IllegalArgumentException("no 'value' line");
        }
        // The payload of a NetworkTables struct:Transform3d topic (PhotonVision's targetPose).
        ByteBuffer buffer = ByteBuffer.allocate(Transform3d.struct.getSize()).order(ByteOrder.LITTLE_ENDIAN);
        Transform3d.struct.pack(buffer, value);
        return buffer.array();
      }
      default -> throw new IllegalArgumentException("unknown message " + message);
    }
  }
}
