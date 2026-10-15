// WPILib Data Log v1.0; source: allwpilib v2026.2.2/wpiutil/doc/datalog.adoc.
export type LogValue = number | string | boolean | number[];
export interface Channel {
  name: string;
  type: string;
  samples: { time: number; value: LogValue }[];
}
export interface Log {
  channels: Channel[];
  start: number;
  end: number;
  warnings: string[];
}
export function readLog(buffer: ArrayBuffer): Log {
  const bytes = new Uint8Array(buffer),
    v = new DataView(buffer),
    decoder = new TextDecoder();
  let pos = 0;
  const need = (n: number) => {
    if (n < 0 || pos + n > bytes.length)
      throw Error(`Truncated WPILOG at byte ${pos}`);
  };
  const uint = (n: number) => {
    need(n);
    let value = 0;
    for (let i = 0; i < n; i++) value += bytes[pos++] * 2 ** (8 * i);
    return value;
  };
  const str = () => {
    const n = uint(4);
    need(n);
    const s = decoder.decode(bytes.subarray(pos, pos + n));
    pos += n;
    return s;
  };
  need(12);
  if (decoder.decode(bytes.subarray(0, 6)) !== "WPILOG")
    throw Error("Not a WPILOG file");
  pos = 6;
  if (uint(2) !== 0x100) throw Error("Unsupported WPILOG version");
  const extra = uint(4);
  need(extra);
  pos += extra;
  const active = new Map<number, Channel>(),
    channels: Channel[] = [],
    warnings = new Set<string>();
  let start = Infinity,
    end = 0;
  while (pos < bytes.length) {
    const bits = uint(1);
    if (bits & 128) throw Error("Invalid WPILOG record header");
    const id = uint((bits & 3) + 1),
      length = uint(((bits >> 2) & 3) + 1),
      time = uint(((bits >> 4) & 7) + 1) / 1e6;
    need(length);
    const begin = pos,
      finish = pos + length;
    if (id === 0) {
      const kind = uint(1);
      if (kind === 0) {
        const entry = uint(4),
          name = str(),
          type = str();
        str();
        if (pos > finish) throw Error("Invalid start record");
        const c = { name, type, samples: [] };
        active.set(entry, c);
        channels.push(c);
      } else if (kind === 1) active.delete(uint(4));
    } else {
      const c = active.get(id);
      if (c) {
        let value: LogValue | undefined;
        const doubleArray = () => {
          if (length % 8) throw Error("Invalid double array length");
          return Array.from({ length: length / 8 }, (_, i) =>
            v.getFloat64(begin + i * 8, true),
          );
        };
        if (c.type === "double" && length === 8)
          value = v.getFloat64(begin, true);
        else if (c.type === "float" && length === 4)
          value = v.getFloat32(begin, true);
        else if (c.type === "int64" && length === 8)
          value = Number(v.getBigInt64(begin, true));
        else if (c.type === "boolean" && length === 1)
          value = bytes[begin] !== 0;
        else if (c.type === "string" || c.type === "json")
          value = decoder.decode(bytes.subarray(begin, finish));
        else if (
          c.type === "double[]" ||
          c.type === "struct:Pose2d" ||
          c.type === "struct:ChassisSpeeds" ||
          c.type === "struct:Rotation2d"
        )
          value = doubleArray();
        else if (c.type !== "structschema")
          warnings.add(
            `Unsupported channel type ${c.type}; retained channel name without decoding.`,
          );
        if (value !== undefined) {
          c.samples.push({ time, value });
          start = Math.min(start, time);
          end = Math.max(end, time);
        }
      }
    }
    pos = finish;
  }
  for (const c of channels) c.samples.sort((a, b) => a.time - b.time);
  return {
    channels,
    start: Number.isFinite(start) ? start : 0,
    end,
    warnings: [...warnings],
  };
}
export function valueAt(
  channel: Channel | undefined,
  time: number,
): LogValue | undefined {
  if (!channel?.samples.length) return;
  const a = channel.samples;
  if (time < a[0].time) return;
  let lo = 0,
    hi = a.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (a[mid].time <= time) lo = mid + 1;
    else hi = mid;
  }
  return a[lo - 1]?.value;
}
export function snapshot(log: Log, time: number): Record<string, LogValue> {
  if (time < log.start || time > log.end)
    throw Error(
      `Choose a timestamp between ${log.start.toFixed(3)} and ${log.end.toFixed(3)} seconds.`,
    );
  const result: Record<string, LogValue> = {};
  for (const c of log.channels) {
    const v = valueAt(c, time);
    if (v !== undefined) result[c.name] = v;
  }
  return result;
}
export function csvRows(text: string): Record<string, string>[] {
  const rows: string[][] = [];
  let row: string[] = [],
    field = "",
    quoted = false;
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (c === '"') {
      if (quoted && text[i + 1] === '"') {
        field += '"';
        i++;
      } else quoted = !quoted;
    } else if (c === "," && !quoted) {
      row.push(field);
      field = "";
    } else if ((c === "\n" || c === "\r") && !quoted) {
      if (c === "\r" && text[i + 1] === "\n") i++;
      row.push(field);
      if (row.some((x) => x !== "")) rows.push(row);
      row = [];
      field = "";
    } else field += c;
  }
  if (quoted) throw Error("Unterminated CSV quote");
  row.push(field);
  if (row.some((x) => x !== "")) rows.push(row);
  const header = rows.shift()?.map((x) => x.trim()) ?? [];
  return rows.map((r) =>
    Object.fromEntries(header.map((h, i) => [h, r[i] ?? ""])),
  );
}
export const csvEncode = (rows: Record<string, unknown>[], headers: string[]) =>
  [headers, ...rows.map((r) => headers.map((h) => r[h] ?? ""))]
    .map((row) =>
      row.map((x) => '"' + String(x).replaceAll('"', '""') + '"').join(","),
    )
    .join("\n");
