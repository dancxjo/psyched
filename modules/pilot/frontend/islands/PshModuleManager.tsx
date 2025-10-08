import { useEffect, useState } from "preact/hooks";

import { Badge, type BadgeTone, Card } from "@pilot/components/dashboard.tsx";

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

type Message = {
  kind: "success" | "error";
  text: string;
};

const MODULE_TONES: Record<ModuleStatus["status"], BadgeTone> = {
  running: "ok",
  stopped: "warn",
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
  const [message, setMessage] = useState<Message | null>(null);

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
        setMessage({ kind: "error", text: res.error ?? "Action failed" });
      } else {
        setMessage({ kind: "success", text: "Action completed successfully" });
        await refresh();
      }
    } catch (error) {
      setMessage({ kind: "error", text: String(error) });
    } finally {
      setLoading(false);
    }
  }

  return (
    <Card
      title="Module lifecycle"
      subtitle="Mirror of psh mod commands"
      tone="amber"
    >
      {message && (
        <p
          class={`note ${
            message.kind === "success" ? "note--success" : "note--alert"
          }`}
        >
          {message.text}
        </p>
      )}
      <div class="button-group button-group--wrap">
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/mod/setup")}
        >
          Setup all
        </button>
        <button
          class="button button--danger"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/mod/teardown")}
        >
          Teardown all
        </button>
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/mod/up")}
        >
          Start all
        </button>
        <button
          class="button button--ghost"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/mod/down")}
        >
          Stop all
        </button>
      </div>
      <div class="table-scroll">
        <table class="data-table">
          <thead>
            <tr>
              <th scope="col">Module</th>
              <th scope="col">Status</th>
              <th scope="col">Actions</th>
            </tr>
          </thead>
          <tbody>
            {statuses.map((status) => (
              <tr key={status.name}>
                <td>{status.name}</td>
                <td>
                  <div class="status-indicator">
                    <Badge
                      label={status.status === "running"
                        ? "Running"
                        : "Stopped"}
                      tone={MODULE_TONES[status.status]}
                    />
                    {status.pid && (
                      <span class="status-indicator__detail">
                        PID {status.pid}
                      </span>
                    )}
                  </div>
                </td>
                <td>
                  <div class="button-group button-group--wrap">
                    <button
                      class="button button--small button--primary"
                      type="button"
                      disabled={loading}
                      onClick={() =>
                        runAction("/api/psh/mod/setup", [status.name])}
                    >
                      Setup
                    </button>
                    <button
                      class="button button--small button--primary"
                      type="button"
                      disabled={loading}
                      onClick={() =>
                        runAction("/api/psh/mod/up", [status.name])}
                    >
                      Start
                    </button>
                    <button
                      class="button button--small button--ghost"
                      type="button"
                      disabled={loading}
                      onClick={() =>
                        runAction("/api/psh/mod/down", [status.name])}
                    >
                      Stop
                    </button>
                    <button
                      class="button button--small button--danger"
                      type="button"
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
      </div>
    </Card>
  );
}
