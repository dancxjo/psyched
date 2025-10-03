import { useState } from "preact/hooks";

type Props = {
  hosts: string[];
};

type ResponseBody = {
  ok: boolean;
  error?: string;
};

export default function PshHostProvisioner({ hosts }: Props) {
  const [selected, setSelected] = useState(hosts[0] ?? "");
  const [message, setMessage] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  async function runProvision() {
    setLoading(true);
    setMessage(null);
    try {
      const res: ResponseBody = await fetch("/api/psh/host/setup", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ hosts: selected ? [selected] : undefined }),
      }).then((r) => r.json());
      if (!res.ok) {
        setMessage(res.error ?? "Provisioning failed");
      } else {
        setMessage(`Provisioning triggered for ${selected || "local host"}.`);
      }
    } catch (error) {
      setMessage(String(error));
    } finally {
      setLoading(false);
    }
  }

  return (
    <section class="psh-card">
      <header>
        <h2>Host Provisioning</h2>
        <p>
          Select a host configuration and run <code>psh host setup</code>.
        </p>
      </header>
      <label>
        Host profile
        <select
          value={selected}
          onChange={(event) =>
            setSelected((event.target as HTMLSelectElement).value)}
          disabled={loading}
        >
          <option value="">Detected hostname</option>
          {hosts.map((host) => <option key={host} value={host}>{host}</option>)}
        </select>
      </label>
      <button disabled={loading} onClick={runProvision}>
        {loading ? "Running…" : "Run Provisioning"}
      </button>
      {message && <p class="psh-message">{message}</p>}
    </section>
  );
}
