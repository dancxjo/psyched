import { useState } from "preact/hooks";

import { Card } from "@pilot/components/dashboard.tsx";

type Props = {
  hosts: string[];
};

type ResponseBody = {
  ok: boolean;
  error?: string;
};

type Message = {
  kind: "success" | "error";
  text: string;
};

export default function PshHostProvisioner({ hosts }: Props) {
  const [selected, setSelected] = useState(hosts[0] ?? "");
  const [message, setMessage] = useState<Message | null>(null);
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
        setMessage({ kind: "error", text: res.error ?? "Provisioning failed" });
      } else {
        setMessage({
          kind: "success",
          text: `Provisioning triggered for ${selected || "local host"}.`,
        });
      }
    } catch (error) {
      setMessage({ kind: "error", text: String(error) });
    } finally {
      setLoading(false);
    }
  }

  return (
    <Card
      title="Host provisioning"
      subtitle="Select a host profile and execute psh host setup"
      tone="teal"
    >
      <div class="form-grid">
        <label class="form-field">
          <span class="form-label">Host profile</span>
          <select
            class="form-control"
            value={selected}
            onChange={(event) =>
              setSelected((event.target as HTMLSelectElement).value)}
            disabled={loading}
          >
            <option value="">Detected hostname</option>
            {hosts.map((host) => <option key={host} value={host}>{host}
            </option>)}
          </select>
        </label>
      </div>
      <div class="button-group">
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={runProvision}
        >
          {loading ? "Running…" : "Run provisioning"}
        </button>
      </div>
      {message && (
        <p
          class={`note ${
            message.kind === "success" ? "note--success" : "note--alert"
          }`}
        >
          {message.text}
        </p>
      )}
    </Card>
  );
}
