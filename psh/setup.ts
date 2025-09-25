export async function setup(options: Record<string, unknown>, args: unknown[]) {
    console.log("setup() called with:");
    console.log("options:", options);
    console.log("args:", args);

    // Example: pretend to do async setup work
    await new Promise((res) => setTimeout(res, 50));

    console.log("Setup complete.");
}
