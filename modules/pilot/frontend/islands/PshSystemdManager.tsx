import { useState } from "preact/hooks";

type Props = {
  modules: string[];
};

type ResponseBody = {
  ok: boolean;
  error?: string;
};

async function post(url: string, module: string) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ module }),
  });
  return response.json() as Promise<ResponseBody>;
}

export default function PshSystemdManager({ modules }: Props) {
  const [module, setModule] = useState(modules[0] ?? "");
  const [message, setMessage] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  async function run(endpoint: string) {
    if (!module) {
      setMessage("Select a module first");
      return;
    }
    setLoading(true);
    setMessage(null);
    try {
      const res = await post(endpoint, module);
      if (!res.ok) {
        setMessage(res.error ?? "Action failed");
      } else {
        setMessage(`Action completed for ${module}`);
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
        <h2>Systemd Units</h2>
        <p>
          Create and control user-level units mapped to module launch scripts.
        </p>
      </header>
      <label>
        Module
        <select
          value={module}
          onChange={(event) =>
            setModule((event.target as HTMLSelectElement).value)}
          disabled={loading}
        >
          {modules.map((name) => <option key={name} value={name}>{name}
          </option>)}
        </select>
      </label>
      <div class="psh-actions">
        <button disabled={loading} onClick={() => run("/api/psh/sys/setup")}>
          Setup
        </button>
        <button disabled={loading} onClick={() => run("/api/psh/sys/teardown")}>
          Teardown
        </button>
        <button disabled={loading} onClick={() => run("/api/psh/sys/enable")}>
          Enable
        </button>
        <button disabled={loading} onClick={() => run("/api/psh/sys/disable")}>
          Disable
        </button>
        <button disabled={loading} onClick={() => run("/api/psh/sys/up")}>
          Start
        </button>
        <button disabled={loading} onClick={() => run("/api/psh/sys/down")}>
          Stop
        </button>
      </div>
      {message && <p class="psh-message">{message}</p>}
    </section>
  );
}
