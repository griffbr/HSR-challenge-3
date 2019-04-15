import { AudioPlayer } from "../audioPlayer/AudioPlayer";
import { Device } from "./Device";
import { SkillSession } from "./SkillSession";
import { User } from "./User";
export declare class SkillContext {
    private _locale;
    private _applicationID?;
    private _accessToken;
    private _apiAccessToken;
    private _apiEndpoint;
    private _device;
    private _user;
    private _session;
    apiAccessToken(): string;
    apiEndpoint(): string;
    applicationID(): string;
    device(): Device;
    user(): User;
    accessToken(): string;
    setAccessToken(token: string): void;
    locale(): string;
    audioPlayer(): AudioPlayer;
    newSession(): void;
    session(): SkillSession;
    endSession(): void;
    activeSession(): boolean;
}
