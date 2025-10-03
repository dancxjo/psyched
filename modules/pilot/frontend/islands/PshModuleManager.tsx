import { useEffect, useState } from "preact/hooks";

type ModuleStatus = {
  name: string;
  status: "running" | "stopped";
  pid?: number;
};

type ApiResponse = {
  ok: boolean;
  statuses?: ModuleStatus[];
  error?: string;
};

type ActionResponse = {
  ok: boolean;
  error?: string;
};

async function post(url: string, body?: unknown) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: body ? JSON.stringify(body) : undefined,
  });
  return response.json();
}

export default function PshModuleManager() {
  const [statuses, setStatuses] = useState<ModuleStatus[]>([]);
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState<string | null>(null);

  async function refresh() {
    const res: ApiResponse = await fetch("/api/psh/mod/list").then((r) =>
      r.json()
    );
    if (res.ok && res.statuses) {
      setStatuses(res.statuses);
    }
  }

  useEffect(() => {
    refresh();
  }, []);

  async function runAction(endpoint: string, modules?: string[]) {
    setLoading(true);
    setMessage(null);
    try {
      const res: ActionResponse = await post(
        endpoint,
        modules?.length ? { modules } : undefined,
      );
      if (!res.ok) {
        setMessage(res.error ?? "Action failed");
      } else {
        setMessage("Action completed successfully");
        await refresh();
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
        <h2>Module Lifecycle</h2>
        <p>
          Mirror of <code>psh mod</code> commands.
        </p>
      </header>
      {message && <p class="psh-message">{message}</p>}
      <div class="psh-actions">
        <button
          disabled={loading}
          onClick={() => runAction("/api/psh/mod/setup")}
        >
          Setup All
        </button>
        <button
          disabled={loading}
          onClick={() => runAction("/api/psh/mod/teardown")}
        >
          Teardown All
        </button>
        <button disabled={loading} onClick={() => runAction("/api/psh/mod/up")}>
          Start All
        </button>
        <button
          disabled={loading}
          onClick={() => runAction("/api/psh/mod/down")}
        >
          Stop All
        </button>
      </div>
      <table class="psh-table">
        <thead>
          <tr>
            <th>Module</th>
            <th>Status</th>
            <th>Actions</th>
          </tr>
        </thead>
        <tbody>
          {statuses.map((status) => (
            <tr key={status.name}>
              <td>{status.name}</td>
              <td>
                {status.status === "running"
                  ? `Running${status.pid ? ` (pid ${status.pid})` : ""}`
                  : "Stopped"}
              </td>
              <td>
                <div class="psh-row-actions">
                  <button
                    disabled={loading}
                    onClick={() =>
                      runAction("/api/psh/mod/setup", [status.name])}
                  >
                    Setup
                  </button>
                  <button
                    disabled={loading}
                    onClick={() => runAction("/api/psh/mod/up", [status.name])}
                  >
                    Start
                  </button>
                  <button
                    disabled={loading}
                    onClick={() =>
                      runAction("/api/psh/mod/down", [status.name])}
                  >
                    Stop
                  </button>
                  <button
                    disabled={loading}
                    onClick={() =>
                      runAction("/api/psh/mod/teardown", [status.name])}
                  >
                    Teardown
                  </button>
                </div>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </section>
  );
}
